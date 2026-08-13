#pragma once

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/UdpSocket.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace aerovista::sync
{
#ifdef WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <winsock2.h>
    using SocketHandle = SOCKET;
#else
    using SocketHandle = int;
#endif

    /// Host-side sync endpoint: UDP sync plane + TCP command listen.
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
            /// Maps to cigi_wire::EyeFrame / AttachState (lla设计 §5).
            bool isLla = false;
        };
        /// Fan-out IGCtrl (+ optional Host eye) to all ready IGs.
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
            SocketHandle tcp = static_cast<SocketHandle>(-1);
            std::string ip;
            uint32_t udpRecvPort = 0;
            bool tcpReady = false;
            bool udpReady = false;
        };

        void acceptLoop();
        void udpLoop();
        void handleClient(SocketHandle client, std::string peerIp);
        void commandReadLoop(SocketHandle client);
        void closeSocket(SocketHandle& s);
        void joinClientThreads();
        int countReadyUnlocked() const;
        void processUdpDatagram(const unsigned char* buf, int n, const char* fromIp);
        void pollUdp();

        // 命令面辅助（初版 §5.2 / §3.1）
        void handleCommandReply(SocketHandle client, const cigi_wire::CommandMsg& msg);
        bool waitReceivedAck(SocketHandle client, std::uint16_t seq, std::uint32_t timeoutMs);
        bool sendAllTcp(SocketHandle s, const void* data, int len);
        void markPeerDisconnected(SocketHandle client);
        void clearReceivedAcks();

        HostConfig _local{};
        SyncPaceConfig _pace{};
        UdpSocket _udp;
        SocketHandle _listenSocket = static_cast<SocketHandle>(-1);

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
        std::mutex _cmdMutex;
        std::condition_variable _cmdCv;
        std::unordered_map<SocketHandle, std::unordered_set<std::uint16_t>> _receivedSeqByPeer;
        mutable std::mutex _resultMutex;
        std::uint16_t _lastResultSeq = 0;
        bool _lastResultAck = false;
        std::function<void(bool ack, std::uint16_t seq, cigi_wire::Command cmd)> _resultCallback;
    };
} // namespace aerovista::sync
