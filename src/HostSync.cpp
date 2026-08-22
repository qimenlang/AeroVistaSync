#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncProtocol.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace aerovista::sync
{
    HostSync::~HostSync()
    {
        shutdown();
    }

    void HostSync::joinClientThreads()
    {
        std::vector<std::thread> threads;
        {
            std::lock_guard lock(_clientThreadsMutex);
            threads.swap(_clientThreads);
        }
        for (auto& t : threads)
        {
            if (t.joinable())
                t.join();
        }
    }

    int HostSync::countReadyUnlocked() const
    {
        int n = 0;
        for (const auto& p : _peers)
        {
            if (p.tcpReady && p.udpReady)
                ++n;
        }
        return n;
    }

    bool HostSync::hasReadyIg() const
    {
        std::lock_guard lock(_peersMutex);
        return countReadyUnlocked() > 0;
    }

    int HostSync::readyIgCount() const
    {
        std::lock_guard lock(_peersMutex);
        return countReadyUnlocked();
    }

    HostStatus HostSync::status() const
    {
        return _status.load();
    }

    std::uint32_t HostSync::igCtrlSentCount() const
    {
        return _frameCounter;
    }

    std::uint32_t HostSync::nextFrameCntr()
    {
        return _frameCounter++;
    }

    std::uint32_t HostSync::sofReceivedCount() const
    {
        const_cast<HostSync*>(this)->pollUdp();
        return _sofReceivedCount.load();
    }

    void HostSync::setPaceConfig(const SyncPaceConfig& pace)
    {
        _pace = pace;
    }

    void HostSync::run()
    {
        _status = HostStatus::RUNNING;
    }

    void HostSync::pollUdp()
    {
        struct Packet
        {
            unsigned char buf[4096]{};
            char fromIp[64]{};
            int n = 0;
        };
        std::vector<Packet> packets;

        {
            std::lock_guard lock(_udpMutex);
            if (!_udp.valid())
                return;

            for (;;)
            {
                Packet p{};
                int fromPort = 0;
                p.n = _udp.recvFrom(p.buf, sizeof(p.buf), p.fromIp, sizeof(p.fromIp), &fromPort);
                if (p.n <= 0)
                    break;
                packets.push_back(p);
            }
        }

        for (const auto& p : packets)
            processUdpDatagram(p.buf, p.n, p.fromIp);
    }

    bool HostSync::initialize(const HostConfig& local)
    {
        shutdown();
        _local = local;
        _status = HostStatus::IDLE;
        _sofReceivedCount = 0;
        _frameCounter = 0;

        std::string udpError;
        if (!_udp.initialize(_local.udpPortSend, _local.udpPortRecv, &udpError))
        {
            std::cerr << "HostSync: UDP open failed: " << udpError << "\n";
            return false;
        }

        std::string tcpError;
        if (!_tcp.listen(_local.tcpPort, &tcpError))
        {
            std::cerr << "HostSync: TCP listen failed on " << _local.tcpPort << ": " << tcpError << "\n";
            _udp.close();
            return false;
        }

        _threadsRunning = true;
        _acceptThread = std::thread(&HostSync::acceptLoop, this);
        _udpThread = std::thread(&HostSync::udpLoop, this);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    void HostSync::shutdown()
    {
        _threadsRunning = false;
        _status = HostStatus::IDLE;
        _tcp.close();

        // 先关 peer socket（唤醒阻塞 recv 的命令读循环），再 join 客户端线程。
        {
            std::lock_guard lock(_peersMutex);
            for (auto& p : _peers)
                if (p.tcp)
                    p.tcp->close();
        }
        if (_acceptThread.joinable())
            _acceptThread.join();
        if (_udpThread.joinable())
            _udpThread.join();
        joinClientThreads();

        {
            std::lock_guard lock(_peersMutex);
            _peers.clear();
            _earlyUdpSyncByPort.clear();
        }

        if (_udp.valid())
            _udp.close();
    }

    void HostSync::acceptLoop()
    {
        while (_threadsRunning)
        {
            auto client = std::make_shared<TcpSocket>();
            std::string peerIp;
            if (!_tcp.accept(*client, &peerIp))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            std::thread worker(&HostSync::handleClient, this, std::move(client), peerIp);
            {
                std::lock_guard lock(_clientThreadsMutex);
                _clientThreads.push_back(std::move(worker));
            }
        }
    }

    void HostSync::handleClient(std::shared_ptr<TcpSocket> client, std::string peerIp)
    {
        sync_proto::WireMsg hello{};
        if (!client->recvAll(&hello, sizeof(hello), 1000) || hello.magic != sync_proto::kMagic ||
            hello.type != static_cast<uint32_t>(sync_proto::MsgType::HELLO))
        {
            return;
        }

        std::uint64_t clientId = 0;
        bool udpAlready = false;
        {
            std::lock_guard lock(_peersMutex);
            clientId = ++_nextClientId;
            IgPeer peer;
            peer.clientId = clientId;
            peer.tcp = client;
            peer.ip = peerIp;
            peer.udpRecvPort = hello.udpRecvPort;
            peer.tcpReady = true;
            udpAlready = _earlyUdpSyncByPort.erase(hello.udpRecvPort) > 0;
            peer.udpReady = udpAlready;
            _peers.push_back(std::move(peer));
        }

        sync_proto::WireMsg ack{};
        ack.magic = sync_proto::kMagic;
        ack.type = static_cast<uint32_t>(sync_proto::MsgType::HELLO_ACK);
        ack.udpRecvPort = static_cast<uint32_t>(_local.udpPortRecv);
        client->sendAll(&ack, sizeof(ack));

        if (udpAlready)
        {
            sync_proto::WireMsg udpAck{};
            udpAck.magic = sync_proto::kMagic;
            udpAck.type = static_cast<uint32_t>(sync_proto::MsgType::UDP_SYNC_ACK);
            udpAck.udpRecvPort = static_cast<uint32_t>(_local.udpPortRecv);
            std::lock_guard lock(_udpMutex);
            _udp.sendTo(peerIp, static_cast<int>(hello.udpRecvPort),
                        reinterpret_cast<const unsigned char*>(&udpAck), sizeof(udpAck));
        }

        commandReadLoop(client, clientId);
        markPeerDisconnected(clientId);
    }

    void HostSync::commandReadLoop(const std::shared_ptr<TcpSocket>& client, std::uint64_t clientId)
    {
        // TCP 读循环（新契约 §5.2）：只做存活检测——recv 对端数据即丢弃；
        // PEER_CLOSED（对端关闭）/ IO_ERROR 即判定断线。命令面 fire-and-forget，无回执解析。
        unsigned char cmdBuf[4096];
        for (;;)
        {
            const RecvOutcome outcome = client->recv(cmdBuf, sizeof(cmdBuf));
            if (outcome.kind == RecvKind::PEER_CLOSED || outcome.kind == RecvKind::IO_ERROR)
                break;
            if (outcome.kind == RecvKind::TIMEOUT)
                continue; // 读超时（SO_RCVTIMEO）≠ 断线
        }
        (void)clientId;
    }

    void HostSync::markPeerDisconnected(std::uint64_t clientId)
    {
        std::lock_guard lock(_peersMutex);
        for (auto it = _peers.begin(); it != _peers.end(); ++it)
        {
            if (it->clientId == clientId)
            {
                if (it->tcp)
                    it->tcp->close();
                _peers.erase(it);
                return;
            }
        }
    }

    void HostSync::processUdpDatagram(const unsigned char* buf, int n, const char* fromIp)
    {
        if (n <= 0)
            return;

        // 握手面（AVSY）。数据面 SOF 是 CIGI（无 AVSY 魔数）。
        if (cigi_wire::isAvsyMagic(buf, n))
        {
            if (n < static_cast<int>(sizeof(sync_proto::WireMsg)))
                return;

            sync_proto::WireMsg header{};
            std::memcpy(&header, buf, sizeof(header));
            if (header.type != static_cast<uint32_t>(sync_proto::MsgType::UDP_SYNC))
                return;

            const uint32_t replyPort = header.udpRecvPort;
            std::string replyIp = fromIp;
            {
                std::lock_guard lock(_peersMutex);
                bool matched = false;
                for (auto& p : _peers)
                {
                    if (p.tcpReady && p.udpRecvPort == header.udpRecvPort)
                    {
                        p.udpReady = true;
                        if (!p.ip.empty())
                            replyIp = p.ip;
                        matched = true;
                        break;
                    }
                }
                if (!matched)
                    _earlyUdpSyncByPort[header.udpRecvPort] = replyIp;
            }

            sync_proto::WireMsg ack{};
            ack.magic = sync_proto::kMagic;
            ack.type = static_cast<uint32_t>(sync_proto::MsgType::UDP_SYNC_ACK);
            ack.udpRecvPort = static_cast<uint32_t>(_local.udpPortRecv);
            {
                std::lock_guard lock(_udpMutex);
                _udp.sendTo(replyIp, static_cast<int>(replyPort),
                            reinterpret_cast<const unsigned char*>(&ack), sizeof(ack));
            }
            return;
        }

        std::uint32_t sofFrame = 0;
        if (cigi_wire::unpackSof(buf, n, sofFrame))
            _sofReceivedCount.fetch_add(1);
    }

    void HostSync::udpLoop()
    {
        while (_threadsRunning)
        {
            pollUdp();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void HostSync::flushTcp()
    {
        if (!_session)
            return;
        CigiOutgoingMsg& omsg = _session->GetOutgoingMsgMgr();
        Cigi_uint8* buf = nullptr;
        int len = 0;
        if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
        {
            omsg.FreeMsg();
            return;
        }

        std::vector<std::shared_ptr<TcpSocket>> targets;
        {
            std::lock_guard lock(_peersMutex);
            for (const auto& p : _peers)
            {
                if (p.tcpReady && p.udpReady && p.tcp)
                    targets.push_back(p.tcp);
            }
        }
        for (const auto& sock : targets)
            sock->sendAll(buf, len);

        omsg.FreeMsg();
    }

    void HostSync::flushUdp()
    {
        if (!_session)
            return;
        CigiOutgoingMsg& omsg = _session->GetOutgoingMsgMgr();
        Cigi_uint8* buf = nullptr;
        int len = 0;
        if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
        {
            omsg.FreeMsg();
            return;
        }

        std::vector<std::pair<std::string, uint32_t>> targets;
        {
            std::lock_guard lock(_peersMutex);
            for (const auto& p : _peers)
            {
                if (p.tcpReady && p.udpReady)
                    targets.emplace_back(p.ip, p.udpRecvPort);
            }
        }
        {
            std::lock_guard lock(_udpMutex);
            for (const auto& t : targets)
                _udp.sendTo(t.first, static_cast<int>(t.second), buf, len);
        }

        omsg.FreeMsg();
    }
} // namespace aerovista::sync
