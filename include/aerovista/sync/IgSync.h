#pragma once

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncConfig.h>
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
#ifdef WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <winsock2.h>
    using IgSocketHandle = SOCKET;
#else
    using IgSocketHandle = int;
#endif

    /// IG-side sync endpoint: connects to Host, UDP sync + TCP command client.
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
            /// From wire AttachState (lla设计 §5); true = Detach+LLA.
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

        /// Consume Host eye received during the last Update (if any).
        std::optional<HostEye> takeReceivedHostEye();

        /// Test / injection: enqueue a Host time stamp as if received this frame.
        /// Phase-unwraps `rawTimeStamp` → `lastSimTimeUs` and records `lastReceivedAtUs`.
        /// Returns true if accepted (frameCntr >= last processed), false if dropped as an old frame.
        bool queueHostTimeStamp(const HostTimeStamp& stamp);

        /// Session reset (design §3): TCP reconnect / Host restart clears phase-unwrap state,
        /// so the next packet starts a fresh absolute base (does not inherit the old large value).
        void resetHostSession();

        /// Most recent Host time stamp converted to us (design §3), 0 if none yet.
        std::uint64_t lastHostSimTimeUs() const;

        /// Current compensated simulation time: internal nowUs = vsg::clock::now().
        std::uint64_t simTimeUs() const;

        /// Compensated simulation time at an explicit monotonic-clock instant (test-controllable).
        std::uint64_t simTimeUsAt(std::uint64_t nowUs) const;

        /// Extrapolate-freeze threshold (design §4.3).
        void setExtrapolateTimeoutUs(std::uint64_t timeoutUs);

        /// Explicit freeze check: nowUs - lastReceivedAtUs > timeout → frozen (design §4.3).
        void updateFreeze(std::uint64_t nowUs);

        /// True once extrapolate timeout exceeded and no new frame arrived.
        bool frozen() const;

        const IgConfig& addressConfig() const { return _local; }

        bool tcpConnected() const;
        bool udpSynced() const;
        IgStatus status() const;

        std::uint32_t igCtrlReceivedCount() const;
        std::uint32_t sofSentCount() const;
        /// FrameCntr from the most recently processed Host IGCtrl (0 if none yet).
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

        void closeTcp();
        void drainUdp();
        bool tcpConnect(const std::string& ip, int port, int timeoutMs);
        bool sendAll(IgSocketHandle s, const void* data, int len);
        bool recvAll(IgSocketHandle s, void* data, int len, int timeoutMs);
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
        IgSocketHandle _tcp = static_cast<IgSocketHandle>(-1);

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
