#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncProtocol.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

namespace aerovista::sync
{
    namespace
    {
        // 本机单调时钟当前时刻，单位 us（时钟同步方案.md §4.2：必须 steady_clock）。
        std::uint64_t nowUs()
        {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        }
    } // namespace

    IgSync::~IgSync()
    {
        shutdown();
    }

    void IgSync::drainUdp()
    {
        unsigned char drain[64];
        while (_udp.recv(drain, sizeof(drain)) > 0)
        {
        }
    }

    void IgSync::markDisconnected()
    {
        _tcp.close();
        _tcpConnected = false;
        _udpSynced = false;
        _status = IgStatus::IDLE;
    }

    bool IgSync::tcpConnected() const
    {
        return _tcpConnected;
    }

    bool IgSync::udpSynced() const
    {
        return _udpSynced;
    }

    bool IgSync::initialize(const IgConfig& local)
    {
        shutdown();
        _local = local;
        _tcpConnected = false;
        _udpSynced = false;
        _status = IgStatus::IDLE;
        _igCtrlReceivedCount = 0;
        _sofSentCount = 0;
        _lastFrameCntr = 0;
        _hasReceivedEye = false;
        _receivedEye = {};
        _hostTarget = {};
        resetHostSession();

        std::string udpError;
        if (!_udp.initialize(_local.udpPortSend, _local.udpPortRecv, &udpError))
        {
            std::cerr << "IgSync: UDP open failed: " << udpError << "\n";
            return false;
        }

        _initialized = true;
        return true;
    }

    void IgSync::shutdown()
    {
        // 先停命令线程（内部 close 唤醒 + join），命令线程退出后 close 收敛到本线程；atomic 兜底。
        stopCommandThread();
        _tcp.close();
        {
            std::lock_guard lock(_pendingMutex);
            _pendingCommands.clear();
        }
        _tcpConnected = false;
        _udpSynced = false;
        _status = IgStatus::IDLE;
        if (_udp.valid())
            _udp.close();
        _initialized = false;
    }

    IgStatus IgSync::status() const
    {
        return _status.load();
    }

    std::uint32_t IgSync::igCtrlReceivedCount() const
    {
        return _igCtrlReceivedCount.load();
    }

    std::uint32_t IgSync::sofSentCount() const
    {
        return _sofSentCount.load();
    }

    std::uint32_t IgSync::lastIgCtrlFrameCntr() const
    {
        return _lastFrameCntr;
    }

    std::optional<IgSync::HostEye> IgSync::takeReceivedHostEye()
    {
        if (!_hasReceivedEye)
            return std::nullopt;
        _hasReceivedEye = false;
        return _receivedEye;
    }

    bool IgSync::queueHostTimeStamp(const HostTimeStamp& stamp)
    {
        // 按 frameCntr 裁决：旧帧整包丢弃（含时间戳）。同帧号重发刷新基准（>= 接受）。
        if (_hasTimeStamp && stamp.frameCntr < _lastFrameCntr)
            return false;

        _lastFrameCntr = stamp.frameCntr;
        applyPhaseUnwrap(stamp.rawTimeStamp);

        _lastSimTimeUs = _extendedTimeTicks * 10; // tick → us
        _lastReceivedAtUs = stamp.receivedAtUs;
        _hasTimeStamp = true;
        _frozen = false;
        return true;
    }

    void IgSync::applyPhaseUnwrap(std::uint32_t raw)
    {
        if (!_hasTimeStamp)
        {
            // 首包：保留 Host 绝对基准（时钟同步方案.md §3），不从 0 起。
            _lastRawTimeStamp = raw;
            _extendedTimeTicks = raw;
            return;
        }
        // 后续包：模减法跨 2^32 回绕（回绕处仍给出正向增量）。
        const std::uint32_t delta = static_cast<std::uint32_t>(raw - _lastRawTimeStamp);
        _extendedTimeTicks += delta;
        _lastRawTimeStamp = raw;
    }

    void IgSync::resetHostSession()
    {
        // 会话重置（TCP 重连 / Host 重启）：相位展开状态从新基准起，不继承旧会话大值。
        _hasTimeStamp = false;
        _lastRawTimeStamp = 0;
        _extendedTimeTicks = 0;
        _lastSimTimeUs = 0;
        _lastReceivedAtUs = 0;
        _frozen = false;
    }

    std::uint64_t IgSync::lastHostSimTimeUs() const
    {
        return _lastSimTimeUs;
    }

    std::uint64_t IgSync::simTimeUs() const
    {
        return simTimeUsAt(nowUs());
    }

    std::uint64_t IgSync::simTimeUsAt(std::uint64_t nowUs) const
    {
        if (!_hasTimeStamp)
            return 0;
        if (_frozen)
            return _lastSimTimeUs;
        return _lastSimTimeUs + (nowUs - _lastReceivedAtUs);
    }

    void IgSync::setExtrapolateTimeoutUs(std::uint64_t timeoutUs)
    {
        _extrapolateTimeoutUs = timeoutUs;
    }

    void IgSync::updateFreeze(std::uint64_t nowUs)
    {
        if (_hasTimeStamp && (nowUs - _lastReceivedAtUs) > _extrapolateTimeoutUs)
            _frozen = true;
    }

    bool IgSync::frozen() const
    {
        return _frozen;
    }

    void IgSync::sendSofPacket(std::uint32_t frameCntr)
    {
        if (_hostTarget.targetAddr.empty())
            return;

        std::vector<unsigned char> sof;
        if (!cigi_wire::packSof(frameCntr, sof))
        {
            std::cerr << "IgSync: CIGI packSof failed\n";
            return;
        }
        _udp.sendTo(_hostTarget.targetAddr, _hostTarget.targetUdpPortRecv, sof.data(),
                    static_cast<int>(sof.size()));
        _sofSentCount.fetch_add(1);
    }

    void IgSync::update(bool sendSof)
    {
        // 无论是否仍连接，都检查外推冻结——Host 离线断线后 IG 仍应每帧检查，
        // 超过阈值后冻结（时间戳停住），而不是随本地流逝无限外推（时钟同步方案.md §4.3）。
        updateFreeze(nowUs());

        if (!_initialized || !_udpSynced)
            return;

        if (_tcpConnected && _udpSynced)
            _status = IgStatus::RUNNING;

        unsigned char buf[4096]{};
        for (;;)
        {
            const int n = _udp.recv(buf, sizeof(buf));
            if (n <= 0)
                break;

            // 握手面（UDP_SYNC_ACK 等）——数据更新时忽略。
            if (cigi_wire::isAvsyMagic(buf, n))
                continue;

            // 命令面：UDP 报文也喂命令面 session，触发业务 processor（摆放/实时位姿）。
            dispatchCommandPlaneFrame(buf, n);

            // 数据面：IGCtrl + 眼点（现有逻辑，全局 CigiRuntime）。
            cigi_wire::HostFrame frame{};
            if (!cigi_wire::unpackHostFrame(buf, n, frame))
                continue;

            // 时间戳相位展开 + 基准更新（时钟同步方案.md §3 / §4）；内含 frameCntr 裁决（旧帧丢弃、同号刷新）。
            const bool accepted = queueHostTimeStamp(HostTimeStamp{frame.frameCntr, frame.timeStamp, nowUs()});
            if (!accepted)
                continue;
            _igCtrlReceivedCount.fetch_add(1);

            if (frame.eye)
            {
                _receivedEye.x = frame.eye->x;
                _receivedEye.y = frame.eye->y;
                _receivedEye.z = frame.eye->z;
                _receivedEye.yawDeg = frame.eye->yawDeg;
                _receivedEye.pitchDeg = frame.eye->pitchDeg;
                _receivedEye.rollDeg = frame.eye->rollDeg;
                _receivedEye.isLla = (frame.eye->frame == cigi_wire::EyeFrame::LLA);
                _hasReceivedEye = true;
            }

            if (sendSof)
                sendSofPacket(_lastFrameCntr);
        }
    }

    void IgSync::dispatchCommandPlaneFrame(const unsigned char* buf, int n)
    {
        if (!_cmdSession)
            return; // 未注册任何 processor：不构造 CCL 会话，跳过命令面解包。
        try
        {
            _cmdSession->GetIncomingMsgMgr().ProcessIncomingMsg(const_cast<unsigned char*>(buf), n);
        }
        catch (...)
        {
            // 命令面解包失败忽略（数据面已处理或非命令面报文）。
        }
    }

    bool IgSync::waitUdpAck(int timeoutMs)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        unsigned char buf[64]{};
        while (std::chrono::steady_clock::now() < deadline)
        {
            const int n = _udp.recv(buf, sizeof(buf));
            if (n >= static_cast<int>(sizeof(sync_proto::WireMsg)))
            {
                sync_proto::WireMsg msg{};
                std::memcpy(&msg, buf, sizeof(msg));
                if (msg.magic == sync_proto::kMagic &&
                    msg.type == static_cast<uint32_t>(sync_proto::MsgType::UDP_SYNC_ACK))
                    return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    bool IgSync::connectOnce(const IgConfig& config)
    {
        // 假定 _tcp 上已连接 TCP。
        sync_proto::WireMsg hello{};
        hello.magic = sync_proto::kMagic;
        hello.type = static_cast<uint32_t>(sync_proto::MsgType::HELLO);
        hello.udpRecvPort = static_cast<uint32_t>(_local.udpPortRecv);
        if (!_tcp.sendAll(&hello, sizeof(hello)))
            return false;

        sync_proto::WireMsg ack{};
        if (!_tcp.recvAll(&ack, sizeof(ack), handshakeTimeoutMs) || ack.magic != sync_proto::kMagic ||
            ack.type != static_cast<uint32_t>(sync_proto::MsgType::HELLO_ACK))
            return false;

        sync_proto::WireMsg udpSync{};
        udpSync.magic = sync_proto::kMagic;
        udpSync.type = static_cast<uint32_t>(sync_proto::MsgType::UDP_SYNC);
        udpSync.udpRecvPort = static_cast<uint32_t>(_local.udpPortRecv);

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(handshakeTimeoutMs);
        while (std::chrono::steady_clock::now() < deadline)
        {
            _udp.sendTo(config.targetAddr, config.targetUdpPortRecv,
                        reinterpret_cast<const unsigned char*>(&udpSync), sizeof(udpSync));
            if (waitUdpAck(50))
                return true;
        }
        return false;
    }

    bool IgSync::connect(const IgConfig& config)
    {
        if (!_initialized)
            return false;

        _tcpConnected = false;
        _udpSynced = false;

    // TCP 重试：Host 可能仍在启动（重连 BDD）。
    // 握手重试（少量）：罕见的 UDP 丢包——错误 UDP 端口快速失败。
        int handshakeFails = 0;
        for (int attempt = 0; attempt < tcpRetryAttempts; ++attempt)
        {
            stopCommandThread();
            _tcp.close();
            drainUdp();

            if (!_tcp.connect(config.targetAddr, config.targetTcpPort, tcpConnectTimeoutMs))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
                continue;
            }

            if (connectOnce(config))
            {
                _hostTarget = config;
                _tcpConnected = true;
                _udpSynced = true;
                _status = IgStatus::RUNNING;
                // 新会话起点：清空相位展开状态（时钟同步方案.md §3）。
                resetHostSession();
                startCommandThread();
                return true;
            }

            _tcp.close();
            if (++handshakeFails >= handshakeRetryAttempts)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        _tcp.close();
        _tcpConnected = false;
        _udpSynced = false;
        return false;
    }

    // =============================================================================
    // 命令面（状态同步设计初版.md §3.1 / §4 / §6）
    // =============================================================================

    void IgSync::startCommandThread()
    {
        stopCommandThread();
        _cmdThreadRunning = true;
        _cmdThread = std::thread(&IgSync::commandLoop, this);
    }

    void IgSync::stopCommandThread()
    {
        if (_cmdThreadRunning.exchange(false))
        {
            // 先置位、再 close 唤醒阻塞在 recv 的命令线程：命令线程醒来后 load() 读到 false，
            // 不再走 markDisconnected 分支，close 因此收敛到调用方线程后再 join。
            // _wsaAcquired 的原子令牌仍兜底并发 close 的 release 唯一性。
            _tcp.close();
            if (_cmdThread.joinable())
                _cmdThread.join();
        }
    }

    void IgSync::commandLoop()
    {
        _tcp.setRecvTimeout(commandRecvTimeoutMs);

        cigi_wire::CigiFrameAssembler assembler;
        unsigned char buf[4096];
        while (_cmdThreadRunning)
        {
            if (!_tcp.valid())
                break;
            const RecvOutcome outcome = _tcp.recv(buf, sizeof(buf));
            if (outcome.kind == RecvKind::PEER_CLOSED || outcome.kind == RecvKind::IO_ERROR)
                break;
            if (outcome.kind == RecvKind::TIMEOUT)
                continue; // 读超时：检查退出标志
            assembler.feed(buf, outcome.bytes, [this](const std::vector<unsigned char>& frame) {
                std::lock_guard lock(_tcpPayloadMutex);
                _tcpPayloadQueue.push_back(frame);
            });
        }
        // recv PEER_CLOSED/错误退出 = Host 断开（TCP 存活检测并入命令读循环线程，§3.3）。
        // 主动 shutdown（_cmdThreadRunning 被置 false）时跳过——shutdown 已处理连接状态。
        if (_cmdThreadRunning.load())
            markDisconnected();
    }

    void IgSync::processCommand(cigi_wire::Command cmd, std::uint16_t seq,
                                const std::vector<std::uint8_t>& payload)
    {
        const std::uint16_t replyCmd = static_cast<std::uint16_t>(cmd);

        bool execute = true;
        {
            std::lock_guard lock(_cmdStateMutex);
            if (_hasCmdState && seq <= _cmdMaxSeq)
                execute = false; // 幂等：重发同 seq 只回 RECEIVED，不重复执行（初版 §2.3）
            else
            {
                _cmdMaxSeq = seq; // 收到即更新
                _hasCmdState = true;
            }
        }
        if (!execute)
        {
            sendCommandReply(static_cast<std::uint16_t>(cigi_wire::kReceivedReplyBase | replyCmd), seq);
            return;
        }

        // 观测（收到即记录，执行入队）：测试断言 commandCount/lastCommandMsgId/lastCommandSeq。
        {
            std::lock_guard lock(_cmdStateMutex);
            _lastCmdMsgId = cmd;
            _lastCmdSeq = seq;
            ++_cmdCount;
        }

        // RECEIVED：命令读循环线程立即回执（可注入延迟复现「回执迟到」），不等待执行。
        if (_cmdReceivedDelayMs > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(_cmdReceivedDelayMs));
        sendCommandReply(static_cast<std::uint16_t>(cigi_wire::kReceivedReplyBase | replyCmd), seq);

        enqueueCommand(cmd, seq, payload);
    }

    void IgSync::enqueueCommand(cigi_wire::Command cmd, std::uint16_t seq,
                                const std::vector<std::uint8_t>& payload)
    {
        PendingCommand pending;
        pending.cmd = cmd;
        pending.seq = seq;
        pending.payload = payload;
        std::lock_guard lock(_pendingMutex);
        _pendingCommands.push_back(std::move(pending));
    }

    void IgSync::runPendingCommands()
    {
        // 主线程：drain 命令面收包队列 → CCL 解包 → 触发业务 processor（§8.2）。
        std::vector<std::vector<unsigned char>> frames;
        {
            std::lock_guard lock(_tcpPayloadMutex);
            frames.swap(_tcpPayloadQueue);
        }
        for (const auto& frame : frames)
        {
            if (!_cmdSession)
                continue; // 未注册 processor：不构造 CCL 会话，丢弃命令面帧。
            try
            {
                _cmdSession->GetIncomingMsgMgr().ProcessIncomingMsg(
                    const_cast<unsigned char*>(frame.data()), static_cast<int>(frame.size()));
            }
            catch (...)
            {
                // 畸形报文忽略；不中断其余报文处理。
            }
        }

        // 旧命令面（待删除）：仍执行旧 pending 命令，保持旧测试（如有）不回归。
        std::vector<PendingCommand> pending;
        {
            std::lock_guard lock(_pendingMutex);
            pending.swap(_pendingCommands);
        }
        for (const auto& cmd : pending)
        {
            const std::uint16_t replyCmd = static_cast<std::uint16_t>(cmd.cmd);
            const bool ack = _cmdHandler ? _cmdHandler(cmd.cmd, cmd.seq, cmd.payload) : true;
            const std::uint16_t resultBase = ack ? cigi_wire::kResultAckBase : cigi_wire::kResultNackBase;
            sendCommandReply(static_cast<std::uint16_t>(resultBase | replyCmd), cmd.seq);
        }
    }

    void IgSync::sendCommandReply(std::uint16_t replyMsgId, std::uint16_t seq)
    {
        cigi_wire::CommandMsg msg;
        msg.msgId = replyMsgId;
        msg.seq = seq;
        std::vector<unsigned char> wire;
        if (!cigi_wire::packCommandMsg(msg, wire))
            return;
        std::lock_guard lock(_cmdSendMutex);
        if (_tcp.valid())
            _tcp.sendAll(wire.data(), static_cast<int>(wire.size()));
    }

    void IgSync::queueCommand(cigi_wire::Command cmd, std::uint16_t seq,
                              const std::vector<std::uint8_t>& payload)
    {
        processCommand(cmd, seq, payload);
    }

    void IgSync::setCommandReceivedDelayMs(std::uint32_t delayMs)
    {
        _cmdReceivedDelayMs = delayMs;
    }

    void IgSync::setCommandHandler(
        std::function<bool(cigi_wire::Command cmd, std::uint16_t seq,
                           const std::vector<std::uint8_t>& payload)>
            handler)
    {
        _cmdHandler = std::move(handler);
    }

    void IgSync::registerEventProcessor(int packetId, CigiBaseEventProcessor* processor)
    {
        ensureCmdSession();
        _cmdSession->GetIncomingMsgMgr().RegisterEventProcessor(packetId, processor);
    }

    cigi_wire::Command IgSync::lastCommandMsgId() const
    {
        std::lock_guard lock(_cmdStateMutex);
        return _lastCmdMsgId;
    }

    std::uint16_t IgSync::lastCommandSeq() const
    {
        std::lock_guard lock(_cmdStateMutex);
        return _lastCmdSeq;
    }

    std::uint32_t IgSync::commandCount() const
    {
        std::lock_guard lock(_cmdStateMutex);
        return _cmdCount;
    }
} // namespace aerovista::sync
