#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/CigiWire.h>

#include "CigiBaseSOF.h"

#include <chrono>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

namespace
{
    constexpr int masterChannelId = 0;

    std::optional<std::chrono::microseconds> ageSince(const std::optional<std::chrono::steady_clock::time_point>& then,
                                                     std::chrono::steady_clock::time_point now)
    {
        if (!then)
            return std::nullopt;
        if (now < *then)
            return std::chrono::microseconds{0};
        return std::chrono::duration_cast<std::chrono::microseconds>(now - *then);
    }

    bool recvCigiHello(aerovista::sync::TcpSocket& client, std::vector<unsigned char>& outFrame)
    {
        client.setRecvTimeout(100);
        aerovista::sync::cigi_wire::CigiFrameAssembler assembler(CIGI_SOF_PACKET_ID_V4);
        unsigned char buf[4096];
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
        while (std::chrono::steady_clock::now() < deadline)
        {
            const aerovista::sync::RecvOutcome outcome = client.recv(buf, sizeof(buf));
            if (outcome.kind == aerovista::sync::RecvKind::PEER_CLOSED ||
                outcome.kind == aerovista::sync::RecvKind::IO_ERROR)
                return false;
            if (outcome.kind == aerovista::sync::RecvKind::TIMEOUT)
                continue;
            assembler.feed(buf, outcome.bytes, [&](const std::vector<unsigned char>& frame) { outFrame = frame; });
            if (!outFrame.empty())
                return true;
        }
        return false;
    }
} // namespace

namespace aerovista::sync
{

    void HostSync::attachCommandCaptures(CigiHostSession& session, std::vector<CigiBaseEventProcessor*>* outRecord)
    {
        auto bind = [&](int packetId, CigiBaseEventProcessor* proc) {
            session.GetIncomingMsgMgr().RegisterEventProcessor(packetId, proc);
            if (outRecord)
                outRecord->push_back(proc);
        };
        bind(CIGI_COLL_DET_VOL_RESP_PACKET_ID_V4, &_collDetVolRespProc);
        bind(CIGI_IG_MSG_PACKET_ID_V4, &_igMsgProc);
        bind(CIGI_EVENT_NOTIFICATION_PACKET_ID_V4, &_eventNotificationProc);
        bind(CIGI_ANIMATION_STOP_PACKET_ID_V4, &_animationStopProc);
        bind(CIGI_HAT_HOT_RESP_PACKET_ID_V4, &_hatHotRespProc);
        bind(CIGI_HAT_HOT_XRESP_PACKET_ID_V4, &_hatHotXRespProc);
        bind(CIGI_LOS_RESP_PACKET_ID_V4, &_losRespProc);
        bind(CIGI_LOS_XRESP_PACKET_ID_V4, &_losXRespProc);
        bind(CIGI_SENSOR_RESP_PACKET_ID_V4, &_sensorRespProc);
        bind(CIGI_SENSOR_XRESP_PACKET_ID_V4, &_sensorXRespProc);
        bind(CIGI_POSITION_RESP_PACKET_ID_V4, &_positionRespProc);
        bind(CIGI_WEATHER_COND_RESP_PACKET_ID_V4, &_weatherCondRespProc);
        bind(CIGI_AEROSOL_RESP_PACKET_ID_V4, &_aerosolRespProc);
        bind(CIGI_MARITIME_SURFACE_RESP_PACKET_ID_V4, &_maritimeSurfaceRespProc);
        bind(CIGI_TERRESTRIAL_SURFACE_RESP_PACKET_ID_V4, &_terrestrialSurfaceRespProc);
        bind(CIGI_COLL_DET_SEG_RESP_PACKET_ID_V4, &_collDetSegRespProc);
    }

    void HostSync::registerUdpProcessors(CigiHostSession& session)
    {
        // 数据面（UDP）：SOF 回显计数（IG 每帧回 SOF，cigi梳理.md §1）。
        // 命令面上报与 TCP 共用同一捕获实例，使 addCallback 对 UDP flush 同样可见（§8.1 对等）；
        // 不写入 _captureProcs，避免同一 Sinkable 被挂两次回调。
        session.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_SOF_PACKET_ID_V4, &_sofProc);
        attachCommandCaptures(session, nullptr);
    }

    void HostSync::registerTcpProcessors(CigiHostSession& session)
    {
        // 命令面（TCP）：SOF 计数（IG TCP 上报消息头也是 SOF）+ 响应/通知/上报类
        //（cigi梳理.md 链路矩阵；VolResp 为既有基础设施）。
        session.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_SOF_PACKET_ID_V4, &_sofProc);
        attachCommandCaptures(session, &_captureProcs);
        _captureProcs.push_back(&_sofProc);
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

    std::vector<IgConnection> HostSync::igSnapshot() const
    {
        std::lock_guard lock(_peersMutex);
        const auto now = std::chrono::steady_clock::now();
        std::vector<IgConnection> rows;
        rows.reserve(_peers.size());
        for (const auto& peer : _peers)
        {
            IgConnection row;
            row.id = peer.clientId;
            row.channelId = peer.channelId;
            row.tcpReady = peer.tcpReady;
            row.udpReady = peer.udpReady;
            row.avgRtt = peer.sofRtt.avgRtt();
            row.lastRtt = peer.sofRtt.lastRtt();
            row.lossRate = peer.sofRtt.lossRate();
            row.sofAge = ageSince(peer.lastSofAt, now);
            rows.push_back(row);
        }
        return rows;
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
        return _sofProc.count.load();
    }

    std::optional<std::chrono::microseconds> HostSync::sofRttLast(std::uint64_t clientId) const
    {
        std::lock_guard lock(_peersMutex);
        for (const auto& peer : _peers)
        {
            if (peer.clientId == clientId)
                return peer.sofRtt.lastRtt();
        }
        return std::nullopt;
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
            int fromPort = 0;
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
                p.n = _udp.recvFrom(p.buf, sizeof(p.buf), p.fromIp, sizeof(p.fromIp), &p.fromPort);
                if (p.n <= 0)
                    break;
                packets.push_back(p);
            }
        }

        for (const auto& p : packets)
            processUdpDatagram(p.buf, p.n, p.fromIp, p.fromPort);
    }

    bool HostSync::initialize(const HostConfig& local)
    {
        shutdown();
        _local = local;
        _status = HostStatus::IDLE;
        _sofProc.count = 0;
        _sofProc.lastFrameCntr = 0;
        _dataFrameCounter = 0;
        _cmdFrameCounter = 0;
        _tcpMsgOpen = false;
        _udpMsgOpen = false;
        _startTime = std::chrono::steady_clock::now();

        std::string udpError;
        if (!_udp.initialize(_local.udpPortRecv, &udpError))
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
        std::vector<unsigned char> helloFrame;
        if (!recvCigiHello(*client, helloFrame))
            return;

        const auto hello = cigi_wire::parseHello(helloFrame.data(), static_cast<int>(helloFrame.size()));
        if (!hello)
            return;

        const auto accepted = tryAddHelloPeer(client, peerIp, hello->udpRecvPort, hello->channelId);
        if (!accepted)
            return;

        if (accepted->second)
            sendUdpSyncAck(peerIp, static_cast<int>(hello->udpRecvPort));

        commandReadLoop(client, accepted->first, hello->channelId);
        markPeerDisconnected(accepted->first);
    }

    void HostSync::enqueueIncomingTcp(int channelId, const std::vector<unsigned char>& frame)
    {
        if (channelId != masterChannelId)
            return;
        std::lock_guard lock(_tcpPayloadMutex);
        _tcpPayloadQueue.push_back(frame);
    }

    void HostSync::commandReadLoop(const std::shared_ptr<TcpSocket>& client, std::uint64_t clientId, int channelId)
    {
        // TCP 读循环：recv → 分帧；仅 master 入队。侧通道仍 recv，只靠 UDP SOF 保活。
        // PEER_CLOSED / IO_ERROR → 断线。HELLO 已在 handleClient 消费，不进队列。
        cigi_wire::CigiFrameAssembler assembler(CIGI_SOF_PACKET_ID_V4);
        unsigned char cmdBuf[4096];
        for (;;)
        {
            const RecvOutcome outcome = client->recv(cmdBuf, sizeof(cmdBuf));
            if (outcome.kind == RecvKind::PEER_CLOSED || outcome.kind == RecvKind::IO_ERROR)
                break;
            if (outcome.kind == RecvKind::TIMEOUT)
                continue; // 读超时（SO_RCVTIMEO）≠ 断线
            assembler.feed(cmdBuf, outcome.bytes, [this, channelId](const std::vector<unsigned char>& frame) {
                enqueueIncomingTcp(channelId, frame);
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

    bool HostSync::channelIdInUseUnlocked(int channelId) const
    {
        for (const auto& peer : _peers)
        {
            if (peer.channelId == channelId)
                return true;
        }
        return false;
    }

    std::optional<std::pair<std::uint64_t, bool>> HostSync::tryAddHelloPeer(
        const std::shared_ptr<TcpSocket>& client, const std::string& peerIp, std::uint32_t udpRecvPort, int channelId)
    {
        std::lock_guard lock(_peersMutex);
        if (channelIdInUseUnlocked(channelId))
            return std::nullopt;

        const std::uint64_t clientId = ++_nextClientId;
        IgPeer peer;
        peer.clientId = clientId;
        peer.tcp = client;
        peer.ip = peerIp;
        peer.udpRecvPort = udpRecvPort;
        peer.channelId = channelId;
        peer.tcpReady = true;
        const auto early = _earlyUdpSyncByPort.find(udpRecvPort);
        const bool udpAlready = early != _earlyUdpSyncByPort.end();
        if (udpAlready)
        {
            peer.udpFromPort = early->second.fromPort;
            peer.udpFromIp = early->second.ip;
            _earlyUdpSyncByPort.erase(early);
        }
        peer.udpReady = udpAlready;
        _peers.push_back(std::move(peer));
        return std::make_pair(clientId, udpAlready);
    }

    std::optional<std::string> HostSync::noteUdpSyncPeer(std::uint32_t udpRecvPort, const std::string& fromIp, int fromPort)
    {
        std::lock_guard lock(_peersMutex);
        for (auto& peer : _peers)
        {
            if (peer.tcpReady && peer.udpRecvPort == udpRecvPort)
            {
                peer.udpReady = true;
                peer.udpFromPort = fromPort;
                peer.udpFromIp = fromIp;
                return peer.ip.empty() ? fromIp : peer.ip;
            }
        }
        _earlyUdpSyncByPort[udpRecvPort] = EarlyUdpSync{fromIp, fromPort};
        return std::nullopt;
    }

    bool HostSync::hasUdpReadyPeer(const std::string& fromIp, int fromPort) const
    {
        std::lock_guard lock(_peersMutex);
        for (const auto& peer : _peers)
        {
            if (peer.udpReady && peer.udpFromPort == fromPort && peer.udpFromIp == fromIp)
                return true;
        }
        return false;
    }

    void HostSync::sendUdpSyncAck(const std::string& ip, int udpRecvPort)
    {
        std::vector<unsigned char> ack;
        if (!cigi_wire::packIgCtrl(0, ack))
            return;
        std::lock_guard lock(_udpMutex);
        _udp.sendTo(ip, udpRecvPort, ack.data(), static_cast<int>(ack.size()));
    }

    void HostSync::recordIgCtrlFanout(std::uint32_t hostFrameNumber, std::chrono::steady_clock::time_point tSend)
    {
        std::lock_guard lock(_peersMutex);
        for (auto& peer : _peers)
        {
            if (peer.tcpReady && peer.udpReady)
                peer.sofRtt.onIgCtrlSent(hostFrameNumber, tSend);
        }
    }

    void HostSync::expireSofRtt(std::chrono::steady_clock::time_point now)
    {
        std::lock_guard lock(_peersMutex);
        for (auto& peer : _peers)
            peer.sofRtt.expire(now);
    }

    void HostSync::ingestUdpSof(const UdpIngress& frame, std::chrono::steady_clock::time_point now)
    {
        const auto countBefore = _sofProc.count.load();
        processIncomingUdpFrame(frame.bytes.data(), static_cast<int>(frame.bytes.size()));
        if (_sofProc.count.load() == countBefore)
            return;

        const auto frameCntr = _sofProc.lastFrameCntr.load();
        std::lock_guard lock(_peersMutex);
        for (auto& peer : _peers)
        {
            if (peer.udpFromPort == frame.fromPort && peer.udpFromIp == frame.fromIp)
            {
                peer.lastSofAt = now;
                peer.sofRtt.onSofReceived(frameCntr, now);
                return;
            }
        }
    }

    void HostSync::processUdpDatagram(const unsigned char* buf, int n, const char* fromIp, int fromPort)
    {
        if (n <= 0)
            return;

        const std::string ip = fromIp ? fromIp : "";
        if (cigi_wire::isSofPacket(buf, n) && !hasUdpReadyPeer(ip, fromPort))
        {
            const auto replyIp = noteUdpSyncPeer(static_cast<std::uint32_t>(fromPort), ip, fromPort);
            if (replyIp)
                sendUdpSyncAck(*replyIp, fromPort);
            return;
        }

        // CIGI 数据报文（SOF / IG 上报等）：I/O 线程只入队，主线程 drainIncoming 解包。
        UdpIngress ingress;
        ingress.bytes.assign(buf, buf + n);
        ingress.fromIp = ip;
        ingress.fromPort = fromPort;
        std::lock_guard lock(_udpPayloadMutex);
        _udpPayloadQueue.push_back(std::move(ingress));
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
        const auto now = std::chrono::steady_clock::now();
        expireSofRtt(now);

        // 按链路喂各 session 解包（§5.1 双 session）：UDP 队列 → _udpSession，TCP 队列 → _tcpSession。
        std::vector<UdpIngress> udpFrames;
        {
            std::lock_guard lock(_udpPayloadMutex);
            udpFrames.swap(_udpPayloadQueue);
        }
        for (const auto& frame : udpFrames)
            ingestUdpSof(frame, now);

        std::vector<std::vector<unsigned char>> tcpFrames;
        {
            std::lock_guard lock(_tcpPayloadMutex);
            tcpFrames.swap(_tcpPayloadQueue);
        }
        for (const auto& frame : tcpFrames)
            processIncomingTcpFrame(frame.data(), static_cast<int>(frame.size()));
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

        fanoutTcp(buf, len);

        omsg.FreeMsg();
    }

    void HostSync::sendTcpMessage(const std::vector<unsigned char>& message)
    {
        if (message.empty())
            return;
        fanoutTcp(message.data(), static_cast<int>(message.size()));
    }

    void HostSync::sendUdpMessage(const std::vector<unsigned char>& message)
    {
        if (message.empty())
            return;
        fanoutUdp(message.data(), static_cast<int>(message.size()));
    }

    std::vector<std::vector<unsigned char>> HostSync::takeIncomingTcp()
    {
        std::vector<std::vector<unsigned char>> tcpFrames;
        {
            std::lock_guard lock(_tcpPayloadMutex);
            tcpFrames.swap(_tcpPayloadQueue);
        }
        return tcpFrames;
    }

    std::vector<std::vector<unsigned char>> HostSync::takeIncomingUdp()
    {
        std::vector<UdpIngress> udpFrames;
        {
            std::lock_guard lock(_udpPayloadMutex);
            udpFrames.swap(_udpPayloadQueue);
        }
        std::vector<std::vector<unsigned char>> out;
        out.reserve(udpFrames.size());
        for (auto& frame : udpFrames)
            out.push_back(std::move(frame.bytes));
        return out;
    }

    void HostSync::fanoutTcp(const unsigned char* buf, int len)
    {
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
    }

    std::size_t HostSync::fanoutUdp(const unsigned char* buf, int len)
    {
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
        return targets.size();
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

        const std::size_t sent = fanoutUdp(buf, len);
        if (sent > 0 && _dataFrameCounter > 0)
            recordIgCtrlFanout(_dataFrameCounter - 1, std::chrono::steady_clock::now());

        omsg.FreeMsg();
    }
} // namespace aerovista::sync
