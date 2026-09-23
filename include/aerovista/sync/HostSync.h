#pragma once

#include <aerovista/sync/CigiIncludes.h>

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/EventProcess.h>
#include <aerovista/sync/SofRttTracker.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SyncInterface.h>
#include <aerovista/sync/TcpSocket.h>
#include <aerovista/sync/UdpSocket.h>

#include "CigiBaseEventProcessor.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"

// IG→Host 方向可达报文的头文件（cigi梳理.md 链路/频率矩阵；SetIncomingHostV4Tbls）。
#include "CigiAerosolRespV4.h"
#include "CigiAnimationStopV4.h"
#include "CigiCollDetSegRespV4.h"
#include "CigiCollDetVolRespV4.h"
#include "CigiEventNotificationV4.h"
#include "CigiHatHotRespV4.h"
#include "CigiHatHotXRespV4.h"
#include "CigiIGMsgV4.h"
#include "CigiLosRespV4.h"
#include "CigiLosXRespV4.h"
#include "CigiMaritimeSurfaceRespV4.h"
#include "CigiPositionRespV4.h"
#include "CigiSensorRespV4.h"
#include "CigiSensorXRespV4.h"
#include "CigiTerrestrialSurfaceRespV4.h"
#include "CigiWeatherCondRespV4.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aerovista::sync
{
    /// Host 侧一条 IG 连接的观测快照（id = Host 分配的 clientId，不是 IG channelId）。
    struct IgConnection
    {
        std::uint64_t id = 0;
        int channelId = 0; ///< HELLO 学到的 IG syncSystem.channelId
        bool tcpReady = false;
        bool udpReady = false;
        /// 滚动平均应用 RTT；不满 10 个匹配样本时为空（SofRttTracker::avgRtt）。
        std::optional<std::chrono::microseconds> avgRtt;
        /// 最近一次匹配的应用 RTT；尚无匹配时为空（SofRttTracker::lastRtt）。
        std::optional<std::chrono::microseconds> lastRtt;
        /// 最近 60 次完成中的超时占比；不满 10 次完成时为空（SofRttTracker::lossRate）。
        std::optional<double> lossRate;
        /// 距上次 UDP SOF 的墙钟间隔；本会话尚未收到该 peer 的 UDP SOF 时为空。
        std::optional<std::chrono::microseconds> sofAge;
    };

    inline bool operator==(const IgConnection& a, const IgConnection& b)
    {
        return a.id == b.id && a.channelId == b.channelId && a.tcpReady == b.tcpReady && a.udpReady == b.udpReady &&
               a.avgRtt == b.avgRtt &&
               a.lastRtt == b.lastRtt && a.lossRate == b.lossRate && a.sofAge == b.sofAge;
    }

    /// Host 侧同步端点：UDP 同步面 + TCP 命令监听。
    class HostSync : public SyncInterface
    {
    public:
        // ===== 门面 =====

        HostSync() = default;
        ~HostSync() override;

        HostSync(const HostSync&) = delete;
        HostSync& operator=(const HostSync&) = delete;

        // ---- 生命周期 ----
        bool initialize(const HostConfig& local);
        void shutdown() override;
        void run();

        // ---- 状态观测 ----
        HostStatus status() const;
        bool hasReadyIg() const;
        int readyIgCount() const;
        /// 当前 peer 表拷贝（含尚未两端 ready 的连接；已摘除的不出现）。
        std::vector<IgConnection> igSnapshot() const;
        /// 本会话已发送的数据面帧数（outMsgWithIgCtrlUdp 每次自动前置 IGCtrl 递增一次）。
        std::uint32_t igCtrlSentCount() const;
        /// 本会话已解包的 SOF 数（观测无副作用；调用方须先 drainIncoming）。
        std::uint32_t sofReceivedCount() const;
        /// 该 peer 最近一次匹配的 IGCtrl↔SOF 应用 RTT（主线程 drain 见到 SOF 的时刻）。
        /// 无匹配样本或未知 clientId 为空。规则见 多通道同步验收测量设计.md §4.5。
        std::optional<std::chrono::microseconds> sofRttLast(std::uint64_t clientId) const;

        // ---- 命令面 / 数据面发送（状态同步设计初版.md §7）：引用式发送接口 ----

        /// TCP 命令面 OutgoingMsg：业务侧 << 报文后调 flushTcp 发送（fire-and-forget）。
        /// 以 IGCtrl 帧头开消息（CCL 要求 Host 消息以 IGCtrl 开头）：帧号 = 命令面计数器、
        /// TimeStampValid=false（命令面不携带有效时间戳）。绑定 _tcpSession（§5.1 双 session）。
        /// 单次 flush 周期内可多次调用填充报文——帧头只填一次（去重），flushTcp 后重置（§7.1）。
        CigiOutgoingMsg& outMsgWithIgCtrlTcp()
        {
            ensureTcpSession();
            auto& omsg = _tcpSession->GetOutgoingMsgMgr();
            if (!_tcpMsgOpen)
            {
                omsg.BeginMsg();
                CigiIGCtrlV4 igCtrl;
                igCtrl.SetFrameCntr(_cmdFrameCounter++);
                igCtrl.SetTimeStampValid(false);
                omsg << igCtrl;
                _tcpMsgOpen = true;
            }
            return omsg;
        }
        /// UDP 数据面 OutgoingMsg：业务侧 << 眼点 << 实时位姿 后调 flushUdp 发送。
        /// 以 IGCtrl 帧头开消息（CCL 要求 Host 消息以 IGCtrl 开头）：帧号 = 数据面计数器，
        /// TimeStamp = HostSync 自计时模拟时间（_startTime = steady_clock::now() 于 initialize；
        /// TimeStamp = (now - _startTime)×100，10µs 步进），TimeStampValid=true（状态同步设计初版.md §7.1）。
        /// 单次 flush 周期内可多次调用填充报文——帧头只填一次（去重），flushUdp 后重置（§7.1）。
        CigiOutgoingMsg& outMsgWithIgCtrlUdp()
        {
            ensureUdpSession();
            auto& omsg = _udpSession->GetOutgoingMsgMgr();
            if (!_udpMsgOpen)
            {
                omsg.BeginMsg();
                CigiIGCtrlV4 igCtrl;
                igCtrl.SetFrameCntr(_dataFrameCounter++);
                igCtrl.SetTimeStamp(cigi_wire::simTimeMsToTimeStamp(currentSimTimeMs()));
                igCtrl.SetTimeStampValid(true);
                omsg << igCtrl;
                _udpMsgOpen = true;
            }
            return omsg;
        }
        void flushTcp() override;
        void flushUdp() override;
        /// 中继：把已切齐的 Host→IG TCP 消息原样 `sendAll` 全体 ready IG（平台同步设计.md §6.2 / §8）。
        /// 不 `PackageMsg`、不加 IGCtrl。空消息跳过。
        void sendTcpMessage(const std::vector<unsigned char>& message) override;
        /// 中继：把已切齐的 Host→IG UDP 数据报原样 `sendto` 全体 ready IG（平台同步设计.md §6.2 / §8）。
        /// 不 `PackageMsg`、不加 IGCtrl。空消息跳过。
        void sendUdpMessage(const std::vector<unsigned char>& message) override;

        // ---- 收包（对等 IG 侧 §8.1）：订阅回调处理 IG→Host 报文 ----

        /// 主线程解包入口：drain UDP/TCP 收包队列 → CCL 解包 → 触发订阅回调。
        /// 业务/测试在需要处理 IG 上报时调用（Host 收包为 push 模式，无独立帧循环）。
        void drainIncoming();
        /// 中继回程：取走已按 SOF 切齐的 TCP 消息字节，不解包（平台同步设计.md §6.3 / §8）。
        /// 与 `drainIncoming` 互斥消费同一 TCP 队列。
        std::vector<std::vector<unsigned char>> takeIncomingTcp() override;
        /// 取走 UDP 数据报字节，不解包（丢弃 fromIp/fromPort）。与 `drainIncoming` 互斥。
        /// 中继不得用本接口把真实 IG SOF 转给平台。
        std::vector<std::vector<unsigned char>> takeIncomingUdp() override;

        /// 注册某类 IG→Host 报文的到达回调：报文解包捕获时同步多播投递（状态同步设计初版.md §8.1）。
        /// 同一类型可注册多个回调（多播，对齐 CCL EventList 多 processor）；捕获时同步调用，
        /// 回调可同步做翻译/合成；回调体捕获对象须存活至 sync 会话结束。
        /// 可在任何时机调用（先于收包）：内部确保会话已创建。
        template <typename PacketT>
        void addCallback(std::function<void(const PacketT&)> callback)
        {
            ensureTcpSession();
            ensureUdpSession();
            for (auto* proc : _captureProcs)
            {
                if (auto* typed = dynamic_cast<Sinkable<PacketT>*>(proc))
                    typed->addCallback(callback);
            }
        }

        // ===== 测试接口 =====

        const HostConfig& addressConfig() const { return _local; }

    private:
        struct IgPeer
        {
            std::uint64_t clientId = 0;
            std::shared_ptr<TcpSocket> tcp;
            std::string ip;
            uint32_t udpRecvPort = 0;
            int channelId = 0;
            /// IG 发送 socket 的源地址/端口（UDP_SYNC 学到）。SOF 按这对字段分到本 peer。
            std::string udpFromIp;
            int udpFromPort = 0;
            bool tcpReady = false;
            bool udpReady = false;
            SofRttTracker sofRtt;
            /// 主线程 drain 见到该 peer UDP SOF 的时刻；未收到过则为空。
            std::optional<std::chrono::steady_clock::time_point> lastSofAt;
        };

        struct EarlyUdpSync
        {
            std::string ip;
            int fromPort = 0;
        };

        struct UdpIngress
        {
            std::vector<unsigned char> bytes;
            std::string fromIp;
            int fromPort = 0;
        };

        void acceptLoop();
        void udpLoop();
        void handleClient(std::shared_ptr<TcpSocket> client, std::string peerIp);
        /// TCP 读循环（peer 线程）：recv → CigiFrameAssembler 分帧 → 入队 tcpPayload；主线程 drainIncoming 解包。
        /// PEER_CLOSED/错误 → markPeerDisconnected（存活检测）。
        void commandReadLoop(const std::shared_ptr<TcpSocket>& client, std::uint64_t clientId);
        void joinClientThreads();
        int countReadyUnlocked() const;
        /// I/O 线程处理一条 UDP 数据报：未 udpReady 的 SOF 当 UDP_SYNC 即时回 IGCtrl ACK；其余入队。
        void processUdpDatagram(const unsigned char* buf, int n, const char* fromIp, int fromPort);
        /// UDP_SYNC：记下 IG 发送源端口，供后续 SOF 按 peer 配对。命中已有 peer 时返回 ACK 目标 IP。
        std::optional<std::string> noteUdpSyncPeer(std::uint32_t udpRecvPort, const std::string& fromIp, int fromPort);
        void sendUdpSyncAck(const std::string& ip, int udpRecvPort);
        bool channelIdInUseUnlocked(int channelId) const;
        bool hasUdpReadyPeer(const std::string& fromIp, int fromPort) const;
        /// HELLO 验收：重复 channelId 拒绝；成功则返回 {clientId, udpAlready}。
        std::optional<std::pair<std::uint64_t, bool>> tryAddHelloPeer(
            const std::shared_ptr<TcpSocket>& client, const std::string& peerIp,
            std::uint32_t udpRecvPort, int channelId);
        void recordIgCtrlFanout(std::uint32_t hostFrameNumber, std::chrono::steady_clock::time_point tSend);
        void expireSofRtt(std::chrono::steady_clock::time_point now);
        void ingestUdpSof(const UdpIngress& frame, std::chrono::steady_clock::time_point now);
        void pollUdp();
        /// 快照 ready peer，逐个 `sendAll`（`flushTcp` 与中继 `sendTcpMessage` 共用）。
        void fanoutTcp(const unsigned char* buf, int len);
        /// 快照 ready peer，逐个 `sendto`（`flushUdp` 与中继 `sendUdpMessage` 共用）。返回发送目标数。
        std::size_t fanoutUdp(const unsigned char* buf, int len);
        /// 主线程解包一条 UDP 报文：_udpSession->ProcessIncomingMsg → 基础设施 + 业务 processor。
        void processIncomingUdpFrame(const unsigned char* buf, int n);
        /// 主线程解包一条 TCP 报文：_tcpSession->ProcessIncomingMsg → 基础设施 + 业务 processor。
        void processIncomingTcpFrame(const unsigned char* buf, int n);

        /// 懒创建 TCP 命令面 CCL 会话（§5.1 双 session）：仅 outMsgWithIgCtrlTcp/flushTcp / TCP 收包首次使用时构造。
        /// CigiSession 构造/析构在 MSVC Debug 下较贵（大 handler 表），纯收包端点不应为此买单。
        /// 基础设施 processor 按链路注册（§8.1）：SOF 计数 TCP+UDP 都注册，碰撞检测响应只注册 TCP。
        void ensureTcpSession()
        {
            if (!_tcpSession)
            {
                _tcpSession = std::make_unique<CigiHostSession>(1, 4096, 1, 4096);
                registerTcpProcessors(*_tcpSession);
            }
        }
        /// 懒创建 UDP 数据面 CCL 会话（§5.1 双 session）：仅 outMsgWithIgCtrlUdp/flushUdp / UDP 收包首次使用时构造。
        /// SOF 计数两个 session 都注册（数据面 SOF 走 UDP，IG TCP 上报消息头也是 SOF）。
        void ensureUdpSession()
        {
            if (!_udpSession)
            {
                _udpSession = std::make_unique<CigiHostSession>(1, 4096, 1, 4096);
                registerUdpProcessors(*_udpSession);
            }
        }
        /// 注册 UDP 数据面可达的 IG→Host 报文捕获（SOF + 与 TCP 共用的命令面捕获镜像）。
        void registerUdpProcessors(CigiHostSession& session);
        /// 注册 TCP 命令面可达的 IG→Host 报文捕获（响应/通知/上报类 + 碰撞检测响应）。
        void registerTcpProcessors(CigiHostSession& session);
        /// 把命令面捕获绑到指定 session。`outRecord` 非空时同时写入 `_captureProcs`（只在 TCP 注册时记一次）。
        void attachCommandCaptures(CigiHostSession& session, std::vector<CigiBaseEventProcessor*>* outRecord);

        void markPeerDisconnected(std::uint64_t clientId);

        /// 当前自计时模拟时间 ms（steady_clock 从 _startTime 起流逝，状态同步设计初版.md §7.1）。
        double currentSimTimeMs() const
        {
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _startTime)
                .count();
        }

        // 基础设施 processor（§8.1 通用模式，统一定义于 EventProcess.h）：
        // SOF 回显计数（IG→Host）。碰撞检测体积响应走通用捕获
        //（PacketCaptureProc<CigiCollDetVolRespV4>，见下方成员 + registerTcpProcessors）。
        SofCaptureProc _sofProc;

        // IG→Host 一次性/响应/通知类（TCP 命令面，cigi梳理.md §1/§3~§7 链路矩阵）。
        // CollDetVolResp 订阅经通用捕获 `addCallback<CigiCollDetVolRespV4>()`。
        PacketCaptureProc<CigiCollDetVolRespV4> _collDetVolRespProc;
        PacketCaptureProc<CigiIGMsgV4> _igMsgProc;
        PacketCaptureProc<CigiEventNotificationV4> _eventNotificationProc;
        PacketCaptureProc<CigiAnimationStopV4> _animationStopProc;
        PacketCaptureProc<CigiHatHotRespV4> _hatHotRespProc;
        PacketCaptureProc<CigiHatHotXRespV4> _hatHotXRespProc;
        PacketCaptureProc<CigiLosRespV4> _losRespProc;
        PacketCaptureProc<CigiLosXRespV4> _losXRespProc;
        PacketCaptureProc<CigiSensorRespV4> _sensorRespProc;
        PacketCaptureProc<CigiSensorXRespV4> _sensorXRespProc;
        PacketCaptureProc<CigiPositionRespV4> _positionRespProc;
        PacketCaptureProc<CigiWeatherCondRespV4> _weatherCondRespProc;
        PacketCaptureProc<CigiAerosolRespV4> _aerosolRespProc;
        PacketCaptureProc<CigiMaritimeSurfaceRespV4> _maritimeSurfaceRespProc;
        PacketCaptureProc<CigiTerrestrialSurfaceRespV4> _terrestrialSurfaceRespProc;
        PacketCaptureProc<CigiCollDetSegRespV4> _collDetSegRespProc;

        /// 全部通用捕获实例的注册表（addCallback<PacketT>() 定位用；注册时填充一次）。
        std::vector<CigiBaseEventProcessor*> _captureProcs;

        HostConfig _local{};
        UdpSocket _udp;
        TcpSocket _tcp;

        std::atomic<bool> _threadsRunning{false};
        std::atomic<HostStatus> _status{HostStatus::IDLE};
        std::thread _acceptThread;
        std::thread _udpThread;

        mutable std::mutex _peersMutex;
        std::vector<IgPeer> _peers;
        std::unordered_map<uint32_t, EarlyUdpSync> _earlyUdpSyncByPort;

        std::mutex _clientThreadsMutex;
        std::vector<std::thread> _clientThreads;

        mutable std::mutex _udpMutex;

        std::uint32_t _dataFrameCounter = 0; ///< 数据面帧号（outMsgWithIgCtrlUdp 自动递增）
        std::uint32_t _cmdFrameCounter = 0;  ///< 命令面帧号（outMsgWithIgCtrlTcp 自动递增，与数据面解耦）
        std::chrono::steady_clock::time_point _startTime{}; ///< 自计时起点（initialize 时记录）
        bool _tcpMsgOpen = false; ///< 当前 TCP 消息已填 IGCtrl 帧头（去重；flushTcp 重置）
        bool _udpMsgOpen = false; ///< 当前 UDP 消息已填 IGCtrl 帧头（去重；flushUdp 重置）
        std::uint64_t _nextClientId = 0;

        // 收包 payload 队列：I/O 线程（udpLoop / commandReadLoop）入队，主线程 drainIncoming 解包。
        // UDP 一条数据报 = 一条 CIGI 消息（无需分帧）；TCP 需分帧（§4.2）。
        std::mutex _udpPayloadMutex;
        std::vector<UdpIngress> _udpPayloadQueue;
        std::mutex _tcpPayloadMutex;
        std::vector<std::vector<unsigned char>> _tcpPayloadQueue;

        // CCL 会话（状态同步设计初版.md §5.1：CCL 单线程化 + 双 session——Host 各链路一套 CigiHostSession）；
        // 懒初始化（ensureTcpSession/ensureUdpSession），堆上分配（CigiSession 内含大 handler 表，栈上会溢出）。
        // 发送隔离：outMsgWithIgCtrlTcp/flushTcp 只操作 _tcpSession，outMsgWithIgCtrlUdp/flushUdp 只操作 _udpSession；
        // 收包按链路喂各 session 解包（UDP 队列 → _udpSession，TCP 队列 → _tcpSession）。
        std::unique_ptr<CigiHostSession> _tcpSession;
        std::unique_ptr<CigiHostSession> _udpSession;
    };
} // namespace aerovista::sync
