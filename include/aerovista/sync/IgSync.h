#pragma once

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/TcpSocket.h>
#include <aerovista/sync/UdpSocket.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
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

        // ===== 命令面（状态同步设计初版.md §6） =====

        /// 测试注入：模拟从 TCP 收到一条命令（cmd, seq, payload），走完整幂等/回执/执行路径。
        void queueCommand(cigi_wire::Command cmd, std::uint16_t seq, const std::vector<std::uint8_t>& payload);

        /// 测试注入：收到命令后延迟 delayMs 再回 RECEIVED（稳定复现 RECEIVED 超时）。
        void setCommandReceivedDelayMs(std::uint32_t delayMs);

        /// 业务执行回调：返回 true=成功（回 RESULT-ACK），false=失败（回 RESULT-NACK）。
        void setCommandHandler(
            std::function<bool(cigi_wire::Command cmd, std::uint16_t seq,
                               const std::vector<std::uint8_t>& payload)>
                handler);

        /// 最近执行的命令（幂等去重后）：测试观测。
        cigi_wire::Command lastCommandMsgId() const;
        std::uint16_t lastCommandSeq() const;
        std::uint32_t commandCount() const;

        /// 主线程每帧执行待办命令（场景归属主线程，状态同步设计初版.md §4）。由 SynchronSystem::update 调用。
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
        /// 相位展开：把 raw（uint32, 10µs tick）累进 64 位单调 extendedTime（时钟同步方案.md §3）。
        void applyPhaseUnwrap(std::uint32_t raw);

        // 命令面（初版 §3.1 / §6）：命令读循环线程回 RECEIVED + 入队；主线程取队列执行 + 回 RESULT。
        void startCommandThread();
        void stopCommandThread();
        void commandLoop();
        void processCommand(cigi_wire::Command cmd, std::uint16_t seq, const std::vector<std::uint8_t>& payload);
        void enqueueCommand(cigi_wire::Command cmd, std::uint16_t seq, const std::vector<std::uint8_t>& payload);
        void sendCommandReply(std::uint16_t replyMsgId, std::uint16_t seq);

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

        // 命令面状态（初版 §2.3 / §6）
        struct PendingCommand
        {
            cigi_wire::Command cmd = cigi_wire::Command::LOAD_MODEL;
            std::uint16_t seq = 0;
            std::vector<std::uint8_t> payload;
        };
        std::thread _cmdThread;
        std::atomic<bool> _cmdThreadRunning{false};
        std::mutex _pendingMutex;
        std::vector<PendingCommand> _pendingCommands; ///< 命令读循环线程入队，主线程 runPendingCommands 取走
        std::mutex _cmdSendMutex;                     ///< 命令读循环线程（RECEIVED）与主线程（RESULT）都写 _tcp

        mutable std::mutex _cmdStateMutex;
        bool _hasCmdState = false;
        std::uint16_t _cmdMaxSeq = 0;
        cigi_wire::Command _lastCmdMsgId = cigi_wire::Command::LOAD_MODEL;
        std::uint16_t _lastCmdSeq = 0;
        std::uint32_t _cmdCount = 0;
        std::uint32_t _cmdReceivedDelayMs = 0;
        std::function<bool(cigi_wire::Command cmd, std::uint16_t seq,
                           const std::vector<std::uint8_t>& payload)>
            _cmdHandler;
    };
} // namespace aerovista::sync
