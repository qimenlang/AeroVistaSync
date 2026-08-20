#pragma once

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/TcpSocket.h>
#include <aerovista/sync/UdpSocket.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace aerovista::sync
{
    /// Host 侧同步端点：UDP 同步面 + TCP 命令监听。
    class HostSync
    {
    public:
        HostSync() = default;
        ~HostSync();

        HostSync(const HostSync&) = delete;
        HostSync& operator=(const HostSync&) = delete;

        bool initialize(const HostConfig& local);
        void shutdown();

        void run();
        struct EyePose
        {
            double x = 0, y = 0, z = 0;
            double yawDeg = 0, pitchDeg = 0, rollDeg = 0;
            /// 映射到 cigi_wire::EyeFrame / AttachState（lla设计 §5）。
            bool isLla = false;
        };
        /// 向所有 ready IG 扇出 IGCtrl（可选带 Host 眼点）。
        void update(double simTimeMs = 0.0, const EyePose* eye = nullptr);
        void setPaceConfig(const SyncPaceConfig& pace);

        const HostConfig& addressConfig() const { return _local; }

        HostStatus status() const;
        bool hasReadyIg() const;
        int readyIgCount() const;

        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

        // ===== 命令面（状态同步设计初版.md §5） =====

        /// 下发命令到所有 ready IG（串行逐 peer）。返回 true 当且仅当
        /// 每个 ready peer 都在 receivedTimeoutMs 内回 RECEIVED（任一超时/无 ready peer → false）。
        bool sendCommand(cigi_wire::Command cmd, const std::vector<std::uint8_t>& payload,
                         std::uint32_t receivedTimeoutMs = 1000);

        /// 最近收到的 RESULT（peer 线程写，测试主线程读）：seq > 0 表示已收到。
        std::uint16_t lastCommandResultSeq() const;
        /// 最近 RESULT 是否为 ACK（true=RESULT-ACK，false=RESULT-NACK）。
        bool lastCommandResultAck() const;

        /// 命令结果回调（初版 §5.1 onCommandResult）：ack=true 成功 / false 失败。
        void setCommandResultCallback(
            std::function<void(bool ack, std::uint16_t seq, cigi_wire::Command cmd)> callback);

    private:
        struct IgPeer
        {
            std::uint64_t clientId = 0;
            std::shared_ptr<TcpSocket> tcp;
            std::string ip;
            uint32_t udpRecvPort = 0;
            bool tcpReady = false;
            bool udpReady = false;
        };

        void acceptLoop();
        void udpLoop();
        void handleClient(std::shared_ptr<TcpSocket> client, std::string peerIp);
        void commandReadLoop(const std::shared_ptr<TcpSocket>& client, std::uint64_t clientId);
        void joinClientThreads();
        int countReadyUnlocked() const;
        void processUdpDatagram(const unsigned char* buf, int n, const char* fromIp);
        void pollUdp();

        // 命令面辅助（初版 §5.2 / §3.1）
        void handleCommandReply(std::uint64_t clientId, const cigi_wire::CommandMsg& msg);
        bool waitReceivedAck(std::uint64_t clientId, std::uint16_t seq, std::uint32_t timeoutMs);
        void markPeerDisconnected(std::uint64_t clientId);
        void clearReceivedAcks();

        HostConfig _local{};
        SyncPaceConfig _pace{};
        UdpSocket _udp;
        TcpSocket _tcp;

        std::atomic<bool> _threadsRunning{false};
        std::atomic<HostStatus> _status{HostStatus::IDLE};
        std::thread _acceptThread;
        std::thread _udpThread;

        mutable std::mutex _peersMutex;
        std::vector<IgPeer> _peers;
        std::unordered_map<uint32_t, std::string> _earlyUdpSyncByPort;

        std::mutex _clientThreadsMutex;
        std::vector<std::thread> _clientThreads;

        mutable std::mutex _udpMutex;

        std::atomic<std::uint32_t> _igCtrlSentCount{0};
        std::atomic<std::uint32_t> _sofReceivedCount{0};
        std::uint32_t _frameCounter = 0;

        // 命令面状态（初版 §5）
        std::uint16_t _cmdSeq = 0;
        std::uint64_t _nextClientId = 0;
        std::mutex _cmdMutex;
        std::condition_variable _cmdCv;
        std::unordered_map<std::uint64_t, std::unordered_set<std::uint16_t>> _receivedSeqByPeer;
        mutable std::mutex _resultMutex;
        std::uint16_t _lastResultSeq = 0;
        bool _lastResultAck = false;
        std::function<void(bool ack, std::uint16_t seq, cigi_wire::Command cmd)> _resultCallback;
    };
} // namespace aerovista::sync
