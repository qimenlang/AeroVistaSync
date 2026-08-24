#pragma once

#include <aerovista/sync/CigiIncludes.h>

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/EventProcess.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/TcpSocket.h>
#include <aerovista/sync/UdpSocket.h>

#include "CigiBaseEventProcessor.h"
#include "CigiIGSession.h"
#include "CigiSOFV4.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace aerovista::sync
{
    /// 收包帧（UDP/TCP 统一 payload 结构）：原始字节 + I/O 线程记录的本机收到时刻
    /// （us，时钟同步方案.md §3——仅 UDP 数据面填充；TCP 命令面忽略）。
    struct IncomingFrame
    {
        std::vector<unsigned char> bytes;
        std::uint64_t receivedAtUs = 0;
    };

    /// IG 侧同步端点：连接 Host，UDP 同步 + TCP 命令客户端。
    class IgSync
    {
    public:
        IgSync() = default;
        ~IgSync();

        IgSync(const IgSync&) = delete;
        IgSync& operator=(const IgSync&) = delete;

        /// Host 每帧 IGCtrl 携带的时间戳信息（时钟同步方案.md §3 / §4）。
        /// `rawTimeStamp` = CIGI IGCtrl.TimeStamp（uint32，10µs tick）；
        /// `receivedAtUs` = 本机单调时钟收到时刻（us）。注入式测试直接传该值；
        /// 真实链路下由 `IgSync::update` 记录 `vsg::clock::now()` 转 us。
        struct HostTimeStamp
        {
            std::uint32_t frameCntr = 0;
            std::uint32_t rawTimeStamp = 0;
            std::uint64_t receivedAtUs = 0;
        };

        bool initialize(const IgConfig& local);
        bool connect(const IgConfig& config);
        void shutdown();

        /// 主线程收包入口（对等 HostSync::drainIncoming）：统一 drain TCP+UDP 收包队列 → CCL 解包。
        /// 无条件 drain（不检查连接状态，与 Host 对等）；收到新 IGCtrl 时回 SOF（sendSof）。
        void drainIncoming(bool sendSof = true);

        /// 帧级维护（不收包）：外推冻结检查 + RUNNING 状态判定。每帧都应调用。
        void update();

        /// 取走上一次 Update 期间收到的 Host 眼点（CCL 报文值拷贝；processor 缓存，§8.1）。
        std::optional<CigiEntityPositionCtrlV4> takeReceivedHostEye();

        /// 取走最近收到的碰撞检测段定义（Host 下发，processor 缓存，§8.1）。
        std::optional<CigiCollDetSegDefV4> takeReceivedCollDetSegDef();
        /// 取走最近收到的碰撞检测体积定义（Host 下发，processor 缓存，§8.1）。
        std::optional<CigiCollDetVolDefV4> takeReceivedCollDetVolDef();

        /// 测试 / 注入：入队一个 Host 时间戳（如同本帧收到）。
        /// 对 `rawTimeStamp` 做相位展开 → `lastSimTimeUs`，并记录 `lastReceivedAtUs`。
        /// 返回 true 表示接受（frameCntr >= 已处理），false 表示作为旧帧丢弃。
        bool queueHostTimeStamp(const HostTimeStamp& stamp);

        /// 会话重置（设计 §3）：TCP 重连 / Host 重启清空相位展开状态，
        /// 使下一个包从新的绝对基准开始（不继承旧的大值）。
        void resetHostSession();

        /// 最近 Host 时间戳换算成 us（设计 §3），无则 0。
        std::uint64_t lastHostSimTimeUs() const;

        /// 当前补偿后的模拟时间：内部 nowUs = vsg::clock::now()。
        std::uint64_t simTimeUs() const;

        /// 在显式单调时钟时刻的补偿模拟时间（测试可控）。
        std::uint64_t simTimeUsAt(std::uint64_t nowUs) const;

        /// 外推-冻结阈值（设计 §4.3）。
        void setExtrapolateTimeoutUs(std::uint64_t timeoutUs);

        /// 显式冻结检查：nowUs - lastReceivedAtUs > 阈值 → 冻结（设计 §4.3）。
        void updateFreeze(std::uint64_t nowUs);

        /// 外推超时且无新帧到达后为 true。
        bool frozen() const;

        const IgConfig& addressConfig() const { return _local; }

        bool tcpConnected() const;
        bool udpSynced() const;
        IgStatus status() const;

        std::uint32_t igCtrlReceivedCount() const;
        std::uint32_t sofSentCount() const;
        /// 最近处理的 Host IGCtrl 的 FrameCntr（无则 0）。
        std::uint32_t lastIgCtrlFrameCntr() const;

        // ===== 命令面（状态同步设计初版.md §8.1） =====

        /// 注册某个 CIGI 报文的业务 EventProcessor（透传到 CCL session 的 RegisterEventProcessor）。
        /// processor 由 engine 层定义；生命周期需覆盖 sync 会话（§8.1）。
        void registerEventProcessor(int packetId, CigiBaseEventProcessor* processor);

        // ===== 发送（对等 Host 侧 §7）：IG 出站消息自动前置 SOF 帧头 =====

        /// TCP 出站 OutgoingMsg：业务侧 << 报文后调 flushTcp 发送（IG→Host 上报/回传）。
        /// 自动前置 CigiSOFV4 帧头（CCL 要求 IG 消息以 SOF 开头），帧号回显最近 IGCtrl。
        /// 绑定 _tcpSession（§5.1 双 session）。
        CigiOutgoingMsg& tcpOutgoing()
        {
            ensureTcpSession();
            auto& omsg = _tcpSession->GetOutgoingMsgMgr();
            omsg.BeginMsg();
            CigiSOFV4 sof;
            sof.SetFrameCntr(_lastFrameCntr);
            omsg << sof;
            return omsg;
        }
        void flushTcp();

        /// UDP 出站 OutgoingMsg：业务侧 << 报文后调 flushUdp 发送（IG→Host UDP 上报/回传）。
        /// 自动前置 CigiSOFV4 帧头（CCL 要求 IG 消息以 SOF 开头）；目标 = Host `targetUdpPortRecv`。
        /// 绑定 _udpSession（§5.1 双 session）。
        CigiOutgoingMsg& udpOutgoing()
        {
            ensureUdpSession();
            auto& omsg = _udpSession->GetOutgoingMsgMgr();
            omsg.BeginMsg();
            CigiSOFV4 sof;
            sof.SetFrameCntr(_lastFrameCntr);
            omsg << sof;
            return omsg;
        }
        void flushUdp();

    private:
        static constexpr int tcpConnectTimeoutMs = 200;
        static constexpr int handshakeTimeoutMs = 1000;
        static constexpr int tcpRetryAttempts = 16;
        static constexpr int handshakeRetryAttempts = 8;
        static constexpr int commandRecvTimeoutMs = 100;

        void drainUdp();
        bool waitUdpAck(int timeoutMs);
        bool connectOnce(const IgConfig& config);
        void sendSofPacket(std::uint32_t frameCntr);
        void markDisconnected();
        /// UDP 生产-消费等待：I/O 线程 1ms 轮询，drain 空队列时按 1ms 步进等待（最多 kMaxUdpDrainWaitMs），
        /// 保证刚发到的数据报当帧可见。仅 IG 侧需要（帧循环主动 drain，区别于 Host push 模式）。
        void waitForUdpFrames(std::vector<IncomingFrame>& out);
        /// 解包一条 TCP 报文（主线程）：`_tcpSession->ProcessIncomingMsg` → 触发基础设施 + 业务 processor。
        /// 不 reset 基础设施捕获（由调用方决定）；畸形报文吞掉不中断。
        void processIncomingFrame(const unsigned char* buf, int n);
        /// 解包一条 UDP 报文（主线程）：先 reset 基础设施捕获，再经 `_udpSession` 解包，随后更新帧号/时间戳/眼点并回 SOF。
        /// `receivedAtUs` 由 UDP I/O 线程在 recv 时刻记录（时钟同步方案.md §3 要求收到时刻）。
        void processIncomingUdp(const unsigned char* buf, int n, std::uint64_t receivedAtUs, bool sendSof);
        /// 相位展开：把 raw（uint32, 10µs tick）累进 64 位单调 extendedTime（时钟同步方案.md §3）。
        void applyPhaseUnwrap(std::uint32_t raw);

        // 数据面 I/O 线程（§5.1）：UDP recv（非阻塞）→ 入队 udpPayload；主线程 update() drain 解包。
        void startUdpThread();
        void stopUdpThread();
        void udpLoop();
        /// 创建 TCP 命令面 IG CCL 会话并注册基础设施 processor（碰撞检测段/体积定义，Host→IG，§8.1 按链路注册）。
        /// 主线程调用；懒创建于 tcpOutgoing/flushTcp / TCP 收包解包。
        void ensureTcpSession()
        {
            if (!_tcpSession)
            {
                _tcpSession = std::make_unique<CigiIGSession>(1, 4096, 1, 4096);
                _tcpSession->GetIncomingMsgMgr().RegisterEventProcessor(
                    CIGI_COLL_DET_SEG_DEF_PACKET_ID_V4, &_segDefProc);
                _tcpSession->GetIncomingMsgMgr().RegisterEventProcessor(
                    CIGI_COLL_DET_VOL_DEF_PACKET_ID_V4, &_volDefProc);
            }
        }
        /// 创建 UDP 数据面 IG CCL 会话并注册基础设施 processor（IGCtrl 帧节拍 / ownship 眼点捕获，§8.1 按链路注册）。
        /// 主线程调用；懒创建于 udpOutgoing/flushUdp / UDP 收包解包。
        void ensureUdpSession()
        {
            if (!_udpSession)
            {
                _udpSession = std::make_unique<CigiIGSession>(1, 4096, 1, 4096);
                _udpSession->GetIncomingMsgMgr().RegisterEventProcessor(CIGI_IG_CTRL_PACKET_ID_V4,
                                                                       &_igCtrlProc);
                _udpSession->GetIncomingMsgMgr().RegisterEventProcessor(
                    CIGI_ENTITY_POSITION_CTRL_PACKET_ID_V4, &_eyeProc);
            }
        }

        // 命令面 I/O 线程（§5.1）：TCP recv + 分帧 → 入队 tcpPayload；主线程 drainIncoming 解包。
        void startCommandThread();
        void stopCommandThread();
        void commandLoop();

        IgConfig _local{};
        IgConfig _hostTarget{};
        UdpSocket _udp;
        TcpSocket _tcp;

        std::atomic<bool> _initialized{false};
        std::atomic<bool> _tcpConnected{false};
        std::atomic<bool> _udpSynced{false};
        std::atomic<IgStatus> _status{IgStatus::IDLE};

        std::atomic<std::uint32_t> _igCtrlReceivedCount{0};
        std::atomic<std::uint32_t> _sofSentCount{0};
        std::uint32_t _lastFrameCntr = 0;

        // 时钟同步（时钟同步方案.md §3 / §4）
        bool _hasTimeStamp = false;
        std::uint32_t _lastRawTimeStamp = 0;          ///< 最近收到 raw（uint32，10µs tick）
        std::uint64_t _extendedTimeTicks = 0;         ///< 相位展开后的 64 位单调 tick
        std::uint64_t _lastSimTimeUs = 0;             ///< = _extendedTimeTicks * 10（us）
        std::uint64_t _lastReceivedAtUs = 0;          ///< 收到该包时的本机单调时钟（us）
        std::uint64_t _extrapolateTimeoutUs = 200000; ///< 默认 200ms
        bool _frozen = false;

        std::thread _cmdThread;
        std::atomic<bool> _cmdThreadRunning{false};

        // CCL 会话（状态同步设计初版.md §5.1：CCL 单线程化 + 双 session——IG 各链路一套 CigiIGSession）；
        // ensureTcpSession/ensureUdpSession 惰性创建，堆上分配（CigiSession 内含大 handler 表，栈上会溢出）。
        // 发送隔离：tcpOutgoing/flushTcp 只操作 _tcpSession，udpOutgoing/flushUdp 只操作 _udpSession；
        // 收包按链路喂各 session 解包（UDP 队列 → _udpSession，TCP 队列 → _tcpSession）。
        std::unique_ptr<CigiIGSession> _tcpSession;
        std::unique_ptr<CigiIGSession> _udpSession;

        // 基础设施 processor（§8.1 通用模式，统一定义于 EventProcess.h）：
        // IGCtrl 帧节拍/时间戳、ownship 眼点、碰撞检测段/体积定义（Host→IG）。
        IgCtrlCaptureProc _igCtrlProc;
        EyeCaptureProc _eyeProc;
        CollDetSegDefProc _segDefProc;
        CollDetVolDefProc _volDefProc;

        // 数据面 I/O 线程（§5.1）：UDP recv → 入队；主线程 update() drain 解包。
        std::thread _udpThread;
        std::atomic<bool> _udpThreadRunning{false};

        // 数据面队列：I/O 线程（udpLoop）入队，主线程（drainIncoming）drain 解包。
        std::mutex _udpPayloadMutex;
        std::vector<IncomingFrame> _udpPayloadQueue;
        // 命令面队列：I/O 线程（commandLoop）分帧入队，主线程（drainIncoming）drain 解包。
        std::mutex _tcpPayloadMutex;
        std::vector<IncomingFrame> _tcpPayloadQueue;
    };
} // namespace aerovista::sync
