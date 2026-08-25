#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncProtocol.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace aerovista::sync
{
    namespace
    {
        /// 注册单个通用捕获 processor（§8.1）：RegisterEventProcessor + 入注册表（供 takeReceived 遍历）。
        template <typename PacketT>
        void registerCapture(CigiHostSession& session, int packetId, PacketCaptureProc<PacketT>& proc,
                             std::vector<CaptureProcBase*>& registry)
        {
            session.GetIncomingMsgMgr().RegisterEventProcessor(packetId, &proc);
            registry.push_back(&proc);
        }
    } // namespace

    void HostSync::registerUdpProcessors(CigiHostSession& session)
    {
        // 数据面（UDP）：SOF 回显计数（IG 每帧回 SOF，cigi梳理.md §1）。
        session.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_SOF_PACKET_ID_V4, &_sofProc);
    }

    void HostSync::registerTcpProcessors(CigiHostSession& session)
    {
        // 命令面（TCP）：SOF 计数（IG TCP 上报消息头也是 SOF）+ 响应/通知/上报类
        //（cigi梳理.md 链路矩阵；VolResp 为既有基础设施）。
        session.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_SOF_PACKET_ID_V4, &_sofProc);
        registerCapture(session, CIGI_COLL_DET_VOL_RESP_PACKET_ID_V4, _collDetVolRespProc, _captureProcs);
        registerCapture(session, CIGI_IG_MSG_PACKET_ID_V4, _igMsgProc, _captureProcs);
        registerCapture(session, CIGI_EVENT_NOTIFICATION_PACKET_ID_V4, _eventNotificationProc, _captureProcs);
        registerCapture(session, CIGI_ANIMATION_STOP_PACKET_ID_V4, _animationStopProc, _captureProcs);
        registerCapture(session, CIGI_HAT_HOT_RESP_PACKET_ID_V4, _hatHotRespProc, _captureProcs);
        registerCapture(session, CIGI_HAT_HOT_XRESP_PACKET_ID_V4, _hatHotXRespProc, _captureProcs);
        registerCapture(session, CIGI_LOS_RESP_PACKET_ID_V4, _losRespProc, _captureProcs);
        registerCapture(session, CIGI_LOS_XRESP_PACKET_ID_V4, _losXRespProc, _captureProcs);
        registerCapture(session, CIGI_SENSOR_RESP_PACKET_ID_V4, _sensorRespProc, _captureProcs);
        registerCapture(session, CIGI_SENSOR_XRESP_PACKET_ID_V4, _sensorXRespProc, _captureProcs);
        registerCapture(session, CIGI_POSITION_RESP_PACKET_ID_V4, _positionRespProc, _captureProcs);
        registerCapture(session, CIGI_WEATHER_COND_RESP_PACKET_ID_V4, _weatherCondRespProc, _captureProcs);
        registerCapture(session, CIGI_AEROSOL_RESP_PACKET_ID_V4, _aerosolRespProc, _captureProcs);
        registerCapture(session, CIGI_MARITIME_SURFACE_RESP_PACKET_ID_V4, _maritimeSurfaceRespProc, _captureProcs);
        registerCapture(session, CIGI_TERRESTRIAL_SURFACE_RESP_PACKET_ID_V4, _terrestrialSurfaceRespProc,
                        _captureProcs);
        registerCapture(session, CIGI_COLL_DET_SEG_RESP_PACKET_ID_V4, _collDetSegRespProc, _captureProcs);
    }

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
        return _dataFrameCounter;
    }

    std::uint32_t HostSync::sofReceivedCount() const
    {
        const_cast<HostSync*>(this)->drainIncoming();
        return _sofProc.count.load();
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
        _sofProc.count = 0;
        _dataFrameCounter = 0;
        _cmdFrameCounter = 0;
        _tcpMsgOpen = false;
        _udpMsgOpen = false;
        _startTime = std::chrono::steady_clock::now();

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
        {
            std::lock_guard lock(_udpPayloadMutex);
            _udpPayloadQueue.clear();
        }
        {
            std::lock_guard lock(_tcpPayloadMutex);
            _tcpPayloadQueue.clear();
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
        // TCP 读循环：recv → 分帧 → 入队 tcpPayload；主线程 drainIncoming 解包（§8.2 对等）。
        // PEER_CLOSED（对端关闭）/ IO_ERROR 即判定断线。
        cigi_wire::CigiFrameAssembler assembler;
        unsigned char cmdBuf[4096];
        for (;;)
        {
            const RecvOutcome outcome = client->recv(cmdBuf, sizeof(cmdBuf));
            if (outcome.kind == RecvKind::PEER_CLOSED || outcome.kind == RecvKind::IO_ERROR)
                break;
            if (outcome.kind == RecvKind::TIMEOUT)
                continue; // 读超时（SO_RCVTIMEO）≠ 断线
            assembler.feed(cmdBuf, outcome.bytes, [this](const std::vector<unsigned char>& frame) {
                std::lock_guard lock(_tcpPayloadMutex);
                _tcpPayloadQueue.push_back(frame);
            });
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

        // CIGI 数据报文（SOF / IG 上报等）：I/O 线程只入队，主线程 drainIncoming 解包。
        {
            std::lock_guard lock(_udpPayloadMutex);
            _udpPayloadQueue.emplace_back(buf, buf + n);
        }
    }

    void HostSync::udpLoop()
    {
        while (_threadsRunning)
        {
            pollUdp();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void HostSync::processIncomingUdpFrame(const unsigned char* buf, int n)
    {
        ensureUdpSession();
        try
        {
            _udpSession->GetIncomingMsgMgr().ProcessIncomingMsg(const_cast<unsigned char*>(buf), n);
        }
        catch (...)
        {
            // 畸形 / 非命令面报文（如握手残留）——忽略。
        }
    }

    void HostSync::processIncomingTcpFrame(const unsigned char* buf, int n)
    {
        ensureTcpSession();
        try
        {
            _tcpSession->GetIncomingMsgMgr().ProcessIncomingMsg(const_cast<unsigned char*>(buf), n);
        }
        catch (...)
        {
            // 畸形 / 非命令面报文（如握手残留）——忽略。
        }
    }

    void HostSync::drainIncoming()
    {
        // 按链路喂各 session 解包（§5.1 双 session）：UDP 队列 → _udpSession，TCP 队列 → _tcpSession。
        std::vector<std::vector<unsigned char>> udpFrames;
        {
            std::lock_guard lock(_udpPayloadMutex);
            udpFrames.swap(_udpPayloadQueue);
        }
        for (const auto& f : udpFrames)
            processIncomingUdpFrame(f.data(), static_cast<int>(f.size()));

        std::vector<std::vector<unsigned char>> tcpFrames;
        {
            std::lock_guard lock(_tcpPayloadMutex);
            tcpFrames.swap(_tcpPayloadQueue);
        }
        for (const auto& f : tcpFrames)
            processIncomingTcpFrame(f.data(), static_cast<int>(f.size()));
    }

    void HostSync::registerEventProcessor(int packetId, CigiBaseEventProcessor* processor)
    {
        // 业务 processor 两个链路都注册（§8.1）：IG 可能经 TCP 或 UDP 发来上报。
        ensureTcpSession();
        ensureUdpSession();
        _tcpSession->GetIncomingMsgMgr().RegisterEventProcessor(packetId, processor);
        _udpSession->GetIncomingMsgMgr().RegisterEventProcessor(packetId, processor);
    }

    void HostSync::flushTcp()
    {
        // 只打包 TCP 命令面 session（§5.1 双 session）：该链路无待发内容时 PackageMsg 失败 → 不发。
        if (!_tcpSession)
            return;
        CigiOutgoingMsg& omsg = _tcpSession->GetOutgoingMsgMgr();
        Cigi_uint8* buf = nullptr;
        int len = 0;
        try
        {
            if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
            {
                omsg.FreeMsg();
                _tcpMsgOpen = false;
                return;
            }
        }
        catch (...)
        {
            // 空缓冲（该链路从未 BeginMsg）→ 不发送任何字节（双 session 隔离的物理保障）。
            _tcpMsgOpen = false;
            return;
        }
        _tcpMsgOpen = false; // 消息已打包：下一轮 outMsgWithIgCtrlTcp 重新填帧头（§7.1 去重）

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
        // 只打包 UDP 数据面 session（§5.1 双 session）：该链路无待发内容时 PackageMsg 失败 → 不发。
        if (!_udpSession)
            return;
        CigiOutgoingMsg& omsg = _udpSession->GetOutgoingMsgMgr();
        Cigi_uint8* buf = nullptr;
        int len = 0;
        try
        {
            if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
            {
                omsg.FreeMsg();
                _udpMsgOpen = false;
                return;
            }
        }
        catch (...)
        {
            // 空缓冲（该链路从未 BeginMsg）→ 不发送任何字节（双 session 隔离的物理保障）。
            _udpMsgOpen = false;
            return;
        }
        _udpMsgOpen = false; // 消息已打包：下一轮 outMsgWithIgCtrlUdp 重新填帧头（§7.1 去重）

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
