#pragma once

#include <aerovista/sync/CigiIncludes.h>

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/TcpSocket.h>
#include <aerovista/sync/UdpSocket.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiBaseEventProcessor.h"
#include "CigiBaseIGCtrl.h"
#include "CigiIGSession.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace aerovista::sync
{
    /// IG 侧同步端点：连接 Host，UDP 同步 + TCP 命令客户端。
    class IgSync
    {
    public:
        IgSync() = default;
        ~IgSync();

        IgSync(const IgSync&) = delete;
        IgSync& operator=(const IgSync&) = delete;

        struct HostEye
        {
            double x = 0, y = 0, z = 0;
            double yawDeg = 0, pitchDeg = 0, rollDeg = 0;
            /// 来自线上 AttachState（lla设计 §5）；true = Detach+LLA。
            bool isLla = false;
        };

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

        void update(bool sendSof = true);

        /// 取走上一次 Update 期间收到的 Host 眼点（若有）。
        std::optional<HostEye> takeReceivedHostEye();

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

        /// 主线程每帧 drain TCP 命令面收包队列 → CCL 解包 → 触发业务 processor。
        /// 由 SynchronSystem::update 调用（场景归属主线程，状态同步设计初版.md §4）。
        void runPendingCommands();

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
        /// 解包一条 UDP 报文（主线程）：基础设施 processor 捕获帧节拍/眼点；收到新 IGCtrl 时回 SOF。
        /// `receivedAtUs` 由 UDP I/O 线程在 recv 时刻记录（时钟同步方案.md §3 要求收到时刻）。
        void processIncomingUdp(const unsigned char* buf, int n, std::uint64_t receivedAtUs, bool sendSof);
        /// 相位展开：把 raw（uint32, 10µs tick）累进 64 位单调 extendedTime（时钟同步方案.md §3）。
        void applyPhaseUnwrap(std::uint32_t raw);

        // 数据面 I/O 线程（§5.1）：UDP recv（非阻塞）→ 入队 udpPayload；主线程 update() drain 解包。
        void startUdpThread();
        void stopUdpThread();
        void udpLoop();
        /// 创建 IG CCL 会话并注册基础设施 processor（IGCtrl 帧节拍 / ownship 眼点捕获）。
        /// 数据面 + 命令面统一经此会话解包（矛盾 A，§8.2）；主线程调用。
        void ensureSession()
        {
            if (!_session)
            {
                _session = std::make_unique<CigiIGSession>(1, 4096, 1, 4096);
                _session->GetIncomingMsgMgr().RegisterEventProcessor(CIGI_IG_CTRL_PACKET_ID_V4,
                                                                    &_igCtrlProc);
                _session->GetIncomingMsgMgr().RegisterEventProcessor(
                    CIGI_ENTITY_POSITION_CTRL_PACKET_ID_V4, &_eyeProc);
            }
        }

        // 命令面 I/O 线程（§5.1）：TCP recv + 分帧 → 入队 tcpPayload；主线程 runPendingCommands 解包。
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
        bool _hasReceivedEye = false;
        HostEye _receivedEye{};

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

        // 数据面 + 命令面统一 CCL 会话（状态同步设计初版.md §8.1 / §5.1）；
        // ensureSession 惰性创建，堆上分配（CigiSession 内含大 handler 表，栈上会溢出）。
        std::unique_ptr<CigiIGSession> _session;

        // 基础设施 processor（sync 库内部注册，§8.1）：IGCtrl 帧节拍/时间戳、ownship 眼点。
        class IgCtrlCaptureProc : public CigiBaseEventProcessor
        {
        public:
            void OnPacketReceived(CigiBasePacket* packet) override;
            void reset()
            {
                got = false;
                frameCntr = 0;
                timeStamp = 0;
                timeStampValid = false;
            }
            bool got = false;
            std::uint32_t frameCntr = 0;
            std::uint32_t timeStamp = 0;
            bool timeStampValid = false;
        };

        class EyeCaptureProc : public CigiBaseEventProcessor
        {
        public:
            void OnPacketReceived(CigiBasePacket* packet) override;
            void reset()
            {
                got = false;
                eye = {};
            }
            bool got = false;
            HostEye eye{};
        };

        IgCtrlCaptureProc _igCtrlProc;
        EyeCaptureProc _eyeProc;

        // 数据面 I/O 线程（§5.1）：UDP recv → 入队；主线程 update() drain 解包。
        std::thread _udpThread;
        std::atomic<bool> _udpThreadRunning{false};

        // UDP payload 队列：一条数据报 = 原始字节 + I/O 线程记录的本机收到时刻（us，时钟同步方案.md §3）。
        struct UdpDatagram
        {
            std::vector<unsigned char> bytes;
            std::uint64_t receivedAtUs = 0;
        };
        std::mutex _udpPayloadMutex;
        std::vector<UdpDatagram> _udpPayloadQueue;

        // 命令面收包队列：I/O 线程（commandLoop）分帧入队，主线程（runPendingCommands）drain 解包。
        std::mutex _tcpPayloadMutex;
        std::vector<std::vector<unsigned char>> _tcpPayloadQueue;
    };
} // namespace aerovista::sync
