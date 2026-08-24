#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncProtocol.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiIGCtrlV4.h"

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
        stopUdpThread();
        _tcp.close();
        {
            std::lock_guard lock(_udpPayloadMutex);
            _udpPayloadQueue.clear();
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

    void IgSync::update()
    {
        // 无论是否仍连接，都检查外推冻结——Host 离线断线后 IG 仍应每帧检查，
        // 超过阈值后冻结（时间戳停住），而不是随本地流逝无限外推（时钟同步方案.md §4.3）。
        updateFreeze(nowUs());

        if (!_initialized || !_udpSynced)
            return;

        if (_tcpConnected && _udpSynced)
            _status = IgStatus::RUNNING;
    }

    void IgSync::drainIncoming(bool sendSof)
    {
        // 主线程收包入口（对等 HostSync::drainIncoming）：统一 drain TCP+UDP 队列 → CCL 解包。
        // 无条件 drain（不检查连接状态，与 Host 对等）。顺序先 UDP 后 TCP：
        // 同帧内 UDP 先更新 _lastFrameCntr，TCP 出站 tcpOutgoing() 的 SOF 帧号才是最新。
        if (!_session)
            ensureSession();

        // UDP 数据面队列。生产-消费：I/O 线程 1ms 轮询，drain 空队列时按 1ms 步进等待
        // （最多 kMaxUdpDrainWaitMs），保证刚发到的数据报在当帧可见；Host 数据到达频率 ≤ 帧率，
        // 正常运行时队列非空即立即处理，等待仅在「Host 未发帧」的空闲期发生。
        constexpr int kMaxUdpDrainWaitMs = 5;
        std::vector<UdpDatagram> datagrams;
        for (int waited = 0; ; waited += 1)
        {
            {
                std::lock_guard lock(_udpPayloadMutex);
                datagrams.swap(_udpPayloadQueue);
            }
            if (!datagrams.empty() || waited >= kMaxUdpDrainWaitMs)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for (const auto& dg : datagrams)
            processIncomingUdp(dg.bytes.data(), static_cast<int>(dg.bytes.size()), dg.receivedAtUs,
                               sendSof);

        // TCP 命令面队列（原 runPendingCommands 职责）。
        std::vector<std::vector<unsigned char>> frames;
        {
            std::lock_guard lock(_tcpPayloadMutex);
            frames.swap(_tcpPayloadQueue);
        }
        for (const auto& frame : frames)
        {
            try
            {
                _session->GetIncomingMsgMgr().ProcessIncomingMsg(
                    const_cast<unsigned char*>(frame.data()), static_cast<int>(frame.size()));
            }
            catch (...)
            {
                // 畸形报文忽略；不中断其余报文处理。
            }
        }
    }

    void IgSync::processIncomingUdp(const unsigned char* buf, int n, std::uint64_t receivedAtUs,
                                    bool sendSof)
    {
        // 数据面 + 命令面统一经 CCL 会话解包（矛盾 A，§8.2）：
        //   基础设施 processor —— IGCtrl（帧号/时间戳）、ownship 眼点（EntityID==0）
        //   业务 processor（engine 注册）—— 命令实体（EntityID!=0）、SymbolTextDefV4
        // 眼点（EntityID==0）也会触发业务 processor，业务侧须按 EntityID==0 过滤（§4.1）。
        _igCtrlProc.reset();
        _eyeProc.reset();
        try
        {
            _session->GetIncomingMsgMgr().ProcessIncomingMsg(const_cast<unsigned char*>(buf), n);
        }
        catch (...)
        {
            return; // 畸形 / 非命令面报文（如握手残留）——忽略。
        }

        if (_igCtrlProc.got)
        {
            _lastFrameCntr = _igCtrlProc.frameCntr;
            // 时间戳相位展开 + 基准更新（时钟同步方案.md §3 / §4）；内含 frameCntr 裁决（旧帧丢弃、同号刷新）。
            const bool accepted = queueHostTimeStamp(
                HostTimeStamp{_igCtrlProc.frameCntr, _igCtrlProc.timeStamp, receivedAtUs});
            if (accepted)
            {
                _igCtrlReceivedCount.fetch_add(1);
                if (sendSof)
                    sendSofPacket(_igCtrlProc.frameCntr);
            }
        }

        if (_eyeProc.got)
        {
            _receivedEye = _eyeProc.eye;
            _hasReceivedEye = true;
        }
    }

    void IgSync::IgCtrlCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        auto* ig = dynamic_cast<CigiIGCtrlV4*>(packet);
        if (!ig)
            return;
        got = true;
        frameCntr = ig->GetFrameCntr();
        timeStamp = ig->GetTimeStamp();
        timeStampValid = ig->GetTimeStampValid();
    }

    void IgSync::EyeCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        auto* ent = dynamic_cast<CigiEntityPositionCtrlV4*>(packet);
        if (!ent || ent->GetEntityID() != 0)
            return; // 仅捕获 ownship（EntityID==0）眼点；命令实体走业务 processor（§4.1）。
        got = true;
        eye.yawDeg = ent->GetYaw();
        eye.pitchDeg = ent->GetPitch();
        eye.rollDeg = ent->GetRoll();
        if (ent->GetAttachState() == CigiBaseEntityPositionCtrl::Detach)
        {
            eye.isLla = true;
            eye.x = ent->GetLat();
            eye.y = ent->GetLon();
            eye.z = ent->GetAlt();
        }
        else
        {
            eye.isLla = false;
            eye.x = ent->GetXoff();
            eye.y = ent->GetYoff();
            eye.z = ent->GetZoff();
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
            stopUdpThread();
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
                startUdpThread();
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
    // 数据面 I/O 线程（状态同步设计初版.md §5.1 / §6）：UDP recv → 入队，主线程解包。
    // =============================================================================

    void IgSync::startUdpThread()
    {
        stopUdpThread();
        _udpThreadRunning = true;
        _udpThread = std::thread(&IgSync::udpLoop, this);
    }

    void IgSync::stopUdpThread()
    {
        if (_udpThreadRunning.exchange(false))
        {
            // UDP 非阻塞 + 短 sleep，置位后最多一个 sleep 周期即退出，无需 close 唤醒。
            if (_udpThread.joinable())
                _udpThread.join();
        }
    }

    void IgSync::udpLoop()
    {
        unsigned char buf[4096];
        while (_udpThreadRunning)
        {
            const int n = _udp.recv(buf, sizeof(buf));
            if (n > 0)
            {
                // 握手面（UDP_SYNC_ACK 等）不入数据面队列。
                if (!cigi_wire::isAvsyMagic(buf, n))
                {
                    UdpDatagram dg;
                    dg.bytes.assign(buf, buf + n);
                    dg.receivedAtUs = nowUs(); // I/O 线程 recv 时刻（时钟同步方案.md §3）
                    std::lock_guard lock(_udpPayloadMutex);
                    _udpPayloadQueue.push_back(std::move(dg));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
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

    void IgSync::registerEventProcessor(int packetId, CigiBaseEventProcessor* processor)
    {
        ensureSession();
        _session->GetIncomingMsgMgr().RegisterEventProcessor(packetId, processor);
    }

    void IgSync::flushTcp()
    {
        ensureSession();
        CigiOutgoingMsg& omsg = _session->GetOutgoingMsgMgr();
        Cigi_uint8* buf = nullptr;
        int len = 0;
        if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
        {
            omsg.FreeMsg();
            return;
        }
        if (_tcp.valid())
            _tcp.sendAll(buf, len);
        omsg.FreeMsg();
    }

    void IgSync::flushUdp()
    {
        ensureSession();
        if (_hostTarget.targetAddr.empty())
            return;
        CigiOutgoingMsg& omsg = _session->GetOutgoingMsgMgr();
        Cigi_uint8* buf = nullptr;
        int len = 0;
        if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
        {
            omsg.FreeMsg();
            return;
        }
        _udp.sendTo(_hostTarget.targetAddr, _hostTarget.targetUdpPortRecv, buf, len);
        omsg.FreeMsg();
    }
} // namespace aerovista::sync
