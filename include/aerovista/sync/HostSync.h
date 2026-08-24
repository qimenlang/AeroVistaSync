#pragma once

#include <aerovista/sync/CigiIncludes.h>

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/EventProcess.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/TcpSocket.h>
#include <aerovista/sync/UdpSocket.h>

#include "CigiBaseEventProcessor.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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

        const HostConfig& addressConfig() const { return _local; }

        HostStatus status() const;
        bool hasReadyIg() const;
        int readyIgCount() const;

        /// 本会话已分配的数据面帧号数（业务侧组装 IGCtrl 时每帧调 nextFrameCntr 一次）。
        /// 矛盾 A 后无 HostSync::update，帧号由业务侧分配，本计数 = 已分配帧号。
        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

        /// 分配下一个数据面帧号（业务侧组装 IGCtrl 时使用；帧号全局递增，含命令面）。
        std::uint32_t nextFrameCntr();

        // ===== 命令面（状态同步设计初版.md §7）：引用式发送接口 =====

        /// TCP 命令面 OutgoingMsg：业务侧 << 报文后调 flushTcp 发送（fire-and-forget）。
        /// 自动前置 IGCtrl 帧头（CCL 要求 Host 消息以 IGCtrl 开头）。
        CigiOutgoingMsg& tcpOutgoing()
        {
            ensureSession();
            auto& omsg = _session->GetOutgoingMsgMgr();
            omsg.BeginMsg();
            CigiIGCtrlV4 igCtrl;
            igCtrl.SetFrameCntr(_frameCounter++);
            igCtrl.SetTimeStampValid(false);
            omsg << igCtrl;
            return omsg;
        }
        /// UDP 数据面 OutgoingMsg：业务侧完整组装（`<< IGCtrl << 眼点 << 实时位姿`）后调
        /// flushUdp 发送（状态同步设计初版.md §7.1：数据面帧节拍由业务侧组装）。
        CigiOutgoingMsg& udpOutgoing()
        {
            ensureSession();
            auto& omsg = _session->GetOutgoingMsgMgr();
            omsg.BeginMsg();
            return omsg;
        }
        void flushTcp();
        void flushUdp();

        // ===== 收包（对等 IG 侧 §8.1）：注册 processor 处理 IG→Host 报文 =====

        /// 注册某个 CIGI 报文的业务 EventProcessor（透传到 CCL session 的 RegisterEventProcessor）。
        /// 处理 IG 经 TCP/UDP 发来的报文（如 IG 发 SymbolTextDefV4 文本指令）。
        /// processor 由业务层定义；生命周期需覆盖 HostSync 会话。
        void registerEventProcessor(int packetId, CigiBaseEventProcessor* processor);

        /// 主线程解包入口：drain UDP/TCP 收包队列 → CCL 解包 → 触发 processor。
        /// 业务/测试在需要处理 IG 上报时调用（Host 收包为 push 模式，无独立帧循环）。
        void drainIncoming();

        /// 取走最近收到的碰撞检测段响应（IG 回发，processor 缓存，§8.1）。
        std::optional<CigiCollDetSegRespV4> takeReceivedCollDetSegResp();
        /// 取走最近收到的碰撞检测体积响应（IG 回发，processor 缓存，§8.1）。
        std::optional<CigiCollDetVolRespV4> takeReceivedCollDetVolResp();

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
        /// TCP 读循环（peer 线程）：recv → CigiFrameAssembler 分帧 → 入队 tcpPayload；主线程 drainIncoming 解包。
        /// PEER_CLOSED/错误 → markPeerDisconnected（存活检测）。
        void commandReadLoop(const std::shared_ptr<TcpSocket>& client, std::uint64_t clientId);
        void joinClientThreads();
        int countReadyUnlocked() const;
        /// I/O 线程处理一条 UDP 数据报：握手面即时回 ACK；CIGI 报文入队 udpPayload（不解包）。
        void processUdpDatagram(const unsigned char* buf, int n, const char* fromIp);
        void pollUdp();
        /// 主线程解包一条报文（UDP/TCP 共用）：_session->ProcessIncomingMsg → 基础设施 + 业务 processor。
        void processIncomingFrame(const unsigned char* buf, int n);

        /// 懒创建 CCL 会话：仅发送接口 / 收包解包首次使用时才构造。
        /// CigiSession 构造/析构在 MSVC Debug 下较贵（大 handler 表），纯收包端点不应为此买单。
        void ensureSession()
        {
            if (!_session)
            {
                _session = std::make_unique<CigiHostSession>(1, 4096, 1, 4096);
                _session->GetIncomingMsgMgr().RegisterEventProcessor(CIGI_SOF_PACKET_ID_V4, &_sofProc);
                _session->GetIncomingMsgMgr().RegisterEventProcessor(
                    CIGI_COLL_DET_SEG_RESP_PACKET_ID_V4, &_segRespProc);
                _session->GetIncomingMsgMgr().RegisterEventProcessor(
                    CIGI_COLL_DET_VOL_RESP_PACKET_ID_V4, &_volRespProc);
            }
        }

        void markPeerDisconnected(std::uint64_t clientId);

        // 基础设施 processor（§8.1 通用模式，统一定义于 EventProcess.h）：
        // SOF 回显计数、碰撞检测段/体积响应（IG→Host）。
        SofCaptureProc _sofProc;
        CollDetSegRespProc _segRespProc;
        CollDetVolRespProc _volRespProc;

        HostConfig _local{};
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

        std::uint32_t _frameCounter = 0; ///< 已分配帧号数（tcpOutgoing/nextFrameCntr 递增）
        std::uint64_t _nextClientId = 0;

        // 收包 payload 队列：I/O 线程（udpLoop / commandReadLoop）入队，主线程 drainIncoming 解包。
        // UDP 一条数据报 = 一条 CIGI 消息（无需分帧）；TCP 需分帧（§4.2）。
        std::mutex _udpPayloadMutex;
        std::vector<std::vector<unsigned char>> _udpPayloadQueue;
        std::mutex _tcpPayloadMutex;
        std::vector<std::vector<unsigned char>> _tcpPayloadQueue;

        // CCL 会话（状态同步设计初版.md §5.1：CCL 单线程化，Host 一套 CigiHostSession）；
        // 懒初始化（ensureSession），堆上分配（CigiSession 内含大 handler 表，栈上会溢出）。
        std::unique_ptr<CigiHostSession> _session;
    };
} // namespace aerovista::sync
