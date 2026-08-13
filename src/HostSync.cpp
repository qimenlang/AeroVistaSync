#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncProtocol.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

#ifdef WIN32
#    include <ws2tcpip.h>
#else
#    include <arpa/inet.h>
#    include <errno.h>
#    include <fcntl.h>
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <unistd.h>
#endif

namespace aerovista::sync
{
    namespace
    {
#ifdef WIN32
        constexpr SocketHandle kInvalid = INVALID_SOCKET;
        bool isValidSock(SocketHandle s)
        {
            return s != INVALID_SOCKET;
        }
#else
        constexpr SocketHandle kInvalid = -1;
        bool isValidSock(SocketHandle s)
        {
            return s >= 0;
        }
#endif
    } // namespace

    HostSync::~HostSync()
    {
        shutdown();
    }

    void HostSync::closeSocket(SocketHandle& s)
    {
        if (!isValidSock(s))
            return;
#ifdef WIN32
        closesocket(s);
#else
        close(s);
#endif
        s = kInvalid;
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
        return _igCtrlSentCount.load();
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

    void HostSync::update(double simTimeMs, const EyePose* eye)
    {
        if (_status.load() != HostStatus::RUNNING)
            return;

        // FreeRun：绝不等 SOF。Barrier 留给后续。
        (void)_pace;

        const std::uint32_t frameCntr = _frameCounter++;
        cigi_wire::EyePose eyeWire{};
        const cigi_wire::EyePose* eyePtr = nullptr;
        if (eye)
        {
            eyeWire.x = eye->x;
            eyeWire.y = eye->y;
            eyeWire.z = eye->z;
            eyeWire.yawDeg = eye->yawDeg;
            eyeWire.pitchDeg = eye->pitchDeg;
            eyeWire.rollDeg = eye->rollDeg;
            eyeWire.frame = eye->isLla ? cigi_wire::EyeFrame::LLA : cigi_wire::EyeFrame::WORLD_LOCAL;
            eyePtr = &eyeWire;
        }

        std::vector<unsigned char> datagram;
        if (!cigi_wire::packHostFrame(frameCntr, simTimeMs, eyePtr, datagram))
        {
            std::cerr << "HostSync: CIGI packHostFrame failed\n";
            return;
        }

        std::vector<std::pair<std::string, uint32_t>> targets;
        {
            std::lock_guard lock(_peersMutex);
            targets.reserve(_peers.size());
            for (const auto& p : _peers)
            {
                if (p.tcpReady && p.udpReady)
                    targets.emplace_back(p.ip, p.udpRecvPort);
            }
        }

        {
            std::lock_guard lock(_udpMutex);
            for (const auto& t : targets)
            {
                _udp.sendTo(t.first, static_cast<int>(t.second), datagram.data(),
                            static_cast<int>(datagram.size()));
            }
        }

        _igCtrlSentCount.fetch_add(1);
    }

    bool HostSync::initialize(const HostConfig& local)
    {
        shutdown();
        _local = local;
        _status = HostStatus::IDLE;
        _igCtrlSentCount = 0;
        _sofReceivedCount = 0;
        _frameCounter = 0;

        std::string udpError;
        if (!_udp.initialize(_local.bindAddr, _local.udpPortSend, _local.udpPortRecv, &udpError))
        {
            std::cerr << "HostSync: UDP open failed: " << udpError << "\n";
            return false;
        }

#ifdef WIN32
        _listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#else
        _listenSocket = socket(AF_INET, SOCK_STREAM, 0);
#endif
        if (!isValidSock(_listenSocket))
        {
            _udp.close();
            return false;
        }

        int yes = 1;
#ifdef WIN32
        setsockopt(_listenSocket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));
#else
        setsockopt(_listenSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<u_short>(_local.tcpPort));
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(_listenSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        {
            std::cerr << "HostSync: TCP bind failed on " << _local.tcpPort << "\n";
            closeSocket(_listenSocket);
            _udp.close();
            return false;
        }

        if (listen(_listenSocket, 16) != 0)
        {
            closeSocket(_listenSocket);
            _udp.close();
            return false;
        }

#ifdef WIN32
        u_long nonBlock = 1;
        ioctlsocket(_listenSocket, FIONBIO, &nonBlock);
#else
        fcntl(_listenSocket, F_SETFL, O_NONBLOCK);
#endif

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
        closeSocket(_listenSocket);

        // 先关 peer socket（唤醒阻塞 recv 的命令读循环），再 join 客户端线程。
        {
            std::lock_guard lock(_peersMutex);
            for (auto& p : _peers)
                closeSocket(p.tcp);
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
        clearReceivedAcks();

        if (_udp.valid())
            _udp.close();
    }

    void HostSync::acceptLoop()
    {
        while (_threadsRunning)
        {
            sockaddr_in clientAddr{};
#ifdef WIN32
            int len = sizeof(clientAddr);
#else
            socklen_t len = sizeof(clientAddr);
#endif
            SocketHandle client = accept(_listenSocket, reinterpret_cast<sockaddr*>(&clientAddr), &len);
            if (!isValidSock(client))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            char ipBuf[64]{};
#ifdef WIN32
            strncpy_s(ipBuf, inet_ntoa(clientAddr.sin_addr), _TRUNCATE);
#else
            std::snprintf(ipBuf, sizeof(ipBuf), "%s", inet_ntoa(clientAddr.sin_addr));
#endif

            std::thread worker(&HostSync::handleClient, this, client, std::string(ipBuf));
            {
                std::lock_guard lock(_clientThreadsMutex);
                _clientThreads.push_back(std::move(worker));
            }
        }
    }

    void HostSync::handleClient(SocketHandle client, std::string peerIp)
    {
#ifdef WIN32
        DWORD timeoutMs = 1000;
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));
#else
        timeval tv{};
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

        sync_proto::WireMsg hello{};
#ifdef WIN32
        const int n = recv(client, reinterpret_cast<char*>(&hello), sizeof(hello), 0);
#else
        const int n = static_cast<int>(::recv(client, &hello, sizeof(hello), 0));
#endif
        if (n != static_cast<int>(sizeof(hello)) || hello.magic != sync_proto::kMagic ||
            hello.type != static_cast<uint32_t>(sync_proto::MsgType::HELLO))
        {
            closeSocket(client);
            return;
        }

        bool udpAlready = false;
        {
            std::lock_guard lock(_peersMutex);
            IgPeer peer;
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
#ifdef WIN32
        send(client, reinterpret_cast<const char*>(&ack), sizeof(ack), 0);
#else
        ::send(client, &ack, sizeof(ack), 0);
#endif

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

        commandReadLoop(client);
        markPeerDisconnected(client);
        closeSocket(client);
    }

    void HostSync::commandReadLoop(SocketHandle client)
    {
        // 命令读循环（初版 §3.1）：持续读 peer TCP → 分帧 → 解析 RECEIVED/RESULT。
        // 本线程是该 socket 的唯一 recv 者；EOF（recv 返回 0）即判定断线。
        cigi_wire::CommandFrameAssembler assembler;
        unsigned char cmdBuf[4096];
        for (;;)
        {
#ifdef WIN32
            const int n = recv(client, reinterpret_cast<char*>(cmdBuf), sizeof(cmdBuf), 0);
            if (n == 0)
                break; // EOF：IG 主动断开
            if (n < 0)
            {
                const int err = WSAGetLastError();
                if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK)
                    continue; // 读超时（SO_RCVTIMEO）≠ 断线
                break;
            }
#else
            const int n = static_cast<int>(::recv(client, cmdBuf, sizeof(cmdBuf), 0));
            if (n == 0)
                break;
            if (n < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                    continue;
                break;
            }
#endif
            assembler.feed(cmdBuf, n, [this, client](const cigi_wire::CommandMsg& msg) {
                handleCommandReply(client, msg);
            });
        }
    }

    void HostSync::handleCommandReply(SocketHandle client, const cigi_wire::CommandMsg& msg)
    {
        const std::uint16_t kind = static_cast<std::uint16_t>(msg.msgId & 0xF000);
        if (kind == cigi_wire::kReceivedReplyBase)
        {
            // RECEIVED：记录该 peer 已确认的 seq，唤醒阻塞等 RECEIVED 的 sendCommand。
            std::lock_guard lock(_cmdMutex);
            _receivedSeqByPeer[client].insert(msg.seq);
            _cmdCv.notify_all();
            return;
        }
        if (kind == cigi_wire::kResultAckBase || kind == cigi_wire::kResultNackBase)
        {
            const bool ack = (kind == cigi_wire::kResultAckBase);
            const auto cmd = static_cast<cigi_wire::Command>(msg.msgId & 0x0FFF);
            std::function<void(bool, std::uint16_t, cigi_wire::Command)> callback;
            {
                std::lock_guard lock(_resultMutex);
                _lastResultSeq = msg.seq;
                _lastResultAck = ack;
                callback = _resultCallback;
            }
            if (callback)
                callback(ack, msg.seq, cmd);
        }
    }

    bool HostSync::sendCommand(cigi_wire::Command cmd, const std::vector<std::uint8_t>& payload,
                               std::uint32_t receivedTimeoutMs)
    {
        std::vector<SocketHandle> targets;
        {
            std::lock_guard lock(_peersMutex);
            for (const auto& p : _peers)
            {
                if (p.tcpReady && p.udpReady)
                    targets.push_back(p.tcp);
            }
        }
        if (targets.empty())
            return false; // 无 ready peer：无人收到命令（初版 §5.2 返回值语义）

        const std::uint16_t seq = static_cast<std::uint16_t>(++_cmdSeq);
        cigi_wire::CommandMsg msg;
        msg.msgId = static_cast<std::uint16_t>(cmd);
        msg.seq = seq;
        msg.payload = payload;
        std::vector<unsigned char> wire;
        if (!cigi_wire::packCommandMsg(msg, wire))
            return false;

        bool allDelivered = true;
        for (const SocketHandle sock : targets)
        {
            if (!sendAllTcp(sock, wire.data(), static_cast<int>(wire.size())))
            {
                allDelivered = false;
                continue;
            }
            if (!waitReceivedAck(sock, seq, receivedTimeoutMs))
                allDelivered = false;
        }
        return allDelivered;
    }

    bool HostSync::waitReceivedAck(SocketHandle client, std::uint16_t seq, std::uint32_t timeoutMs)
    {
        std::unique_lock lock(_cmdMutex);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        while (_receivedSeqByPeer[client].count(seq) == 0)
        {
            if (_cmdCv.wait_until(lock, deadline) == std::cv_status::timeout)
                return _receivedSeqByPeer[client].count(seq) > 0;
        }
        return true;
    }

    bool HostSync::sendAllTcp(SocketHandle s, const void* data, int len)
    {
        const char* p = static_cast<const char*>(data);
        int sent = 0;
        while (sent < len)
        {
#ifdef WIN32
            const int n = send(s, p + sent, len - sent, 0);
#else
            const int n = static_cast<int>(::send(s, p + sent, static_cast<size_t>(len - sent), 0));
#endif
            if (n <= 0)
                return false;
            sent += n;
        }
        return true;
    }

    void HostSync::markPeerDisconnected(SocketHandle client)
    {
        std::lock_guard lock(_peersMutex);
        for (auto it = _peers.begin(); it != _peers.end(); ++it)
        {
            if (it->tcp == client)
            {
                closeSocket(it->tcp);
                _peers.erase(it);
                return;
            }
        }
    }

    void HostSync::clearReceivedAcks()
    {
        std::lock_guard lock(_cmdMutex);
        _receivedSeqByPeer.clear();
    }

    std::uint16_t HostSync::lastCommandResultSeq() const
    {
        std::lock_guard lock(_resultMutex);
        return _lastResultSeq;
    }

    bool HostSync::lastCommandResultAck() const
    {
        std::lock_guard lock(_resultMutex);
        return _lastResultAck;
    }

    void HostSync::setCommandResultCallback(
        std::function<void(bool ack, std::uint16_t seq, cigi_wire::Command cmd)> callback)
    {
        std::lock_guard lock(_resultMutex);
        _resultCallback = std::move(callback);
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
} // namespace aerovista::sync
