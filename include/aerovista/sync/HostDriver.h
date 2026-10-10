#pragma once

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/HostDataManager.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aerovista::sync
{
    class IgSync;

    /// 持有 `HostSync` + `HostDataManager`；中继时另持虚 `IgSync`。
    /// viewhost 直接使用。不并入 `HostSync`（权威表不进传输类）。
    class HostDriver
    {
    public:
        HostDriver();
        ~HostDriver();

        HostDriver(const HostDriver&) = delete;
        HostDriver& operator=(const HostDriver&) = delete;
        HostDriver(HostDriver&&) = delete;
        HostDriver& operator=(HostDriver&&) = delete;

        /// initialize + run（置 RUNNING）。失败时 error 带上下文。
        /// `relay.enable=true` 时只 listen 真实 IG，不在此时连平台。
        bool initialize(const HostConfig& config, std::string* error = nullptr);
        void shutdown();

        /// 扇出一帧（IGCtrl 由 outMsgWithIgCtrlUdp() 自动前置）+ 可选眼点 → flushUdp。
        /// 中继且转发开着时无操作。关掉转发后用调用方眼点自己组数据面 IGCtrl（平台同步设计.md §6.1）。
        void update(const cigi_wire::EyePose* eye);

        bool loadEntityCatalog(const std::string& path, std::string* error = nullptr);
        std::vector<EntityAuthorityRow> entitySnapshot() const;
        std::optional<EntityAuthorityRow> entityRow(std::uint16_t entityId) const;

        bool setEntityCtrl(std::uint16_t entityId, EntityAuthorityState state, std::uint8_t alpha,
                           std::string* error = nullptr);
        bool setEntityPose(std::uint16_t entityId, const EntityAuthorityPose& pose, std::string* error = nullptr);

        struct EntitySend
        {
            bool entityCtrl = false;
            bool entityPosition = false;
        };
        bool sendEntity(std::uint16_t entityId, EntitySend send, std::string* error = nullptr);

        void pollIncoming();
        /// 中继：起齐后门闩；已连平台则 UI 定时器取出切齐字节原样转发（平台同步设计.md §6.1 / §11.1）。
        /// 回程 TCP 队列只入 master，本类原样 take/send。master UDP 业务包经虚 IG `sendUdpAfterSof`。本地调试无操作。
        void pollRelay();
        /// 运行期转发开关（≠ `relay.enable`）。缓存还没有时关失败并保持转发。
        bool setRelayForwarding(bool forwarding);
        bool relayForwarding() const;
        /// 关成功后应写入 viewhost 当前眼点的那一拍缓存。未关成功时为空。
        std::optional<cigi_wire::EyePose> relayEye() const;
        bool virtualIgHandshakeDone() const;

        template<typename PacketT>
        void addCallback(std::function<void(const PacketT&)> callback)
        {
            _host.addCallback<PacketT>(std::move(callback));
        }

        bool isRunning() const;
        int readyIgCount() const;
        std::vector<IgConnection> igSnapshot() const;
        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

        /// 报文自检：转调 `sendRandomTcpProbe`（viewhost设计.md §4.7）。
        std::string sendRandomTcpPacket();
        /// 报文自检：转调 `sendRandomUdpProbe`。
        std::string sendRandomUdpPacket();
        /// 命令面文本指令：组 `CigiSymbolTextDefV4` 经 TCP flush。空串失败。
        bool sendSymbolText(const std::string& text, std::string* error = nullptr);

    private:
        bool shouldConnectVirtualIg() const;
        void connectVirtualIgToPlatform();
        void forwardFromPlatform();
        void forwardToPlatform();
        void dropQueuedRelay();
        void rememberForwardedOwnship(const std::vector<unsigned char>& dgram);

        HostSync _host;
        HostDataManager _data;
        RelayConfig _relay{};
        std::optional<IgConfig> _igConfig;
        std::unique_ptr<IgSync> _virtualIg;
        std::optional<cigi_wire::EyePose> _cachedOwnship;
        bool _relayForwarding = true;
        bool _initialized = false;
    };
} // namespace aerovista::sync
