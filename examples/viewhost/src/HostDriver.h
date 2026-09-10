#pragma once

#include <aerovista/sync/HostDataManager.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace aerovista::viewhost
{
    /// 持有 HostSync + HostDataManager：生命周期 + 帧驱动 + 意图 API（viewhost设计.md §4.0）。
    class HostDriver
    {
    public:
        HostDriver() = default;
        ~HostDriver();

        HostDriver(const HostDriver&) = delete;
        HostDriver& operator=(const HostDriver&) = delete;

        /// initialize + run（置 RUNNING）。失败时 error 带上下文。
        bool initialize(const aerovista::sync::HostConfig& config, std::string* error = nullptr);
        void shutdown();

        /// 扇出一帧（IGCtrl 由 outMsgWithIgCtrlUdp() 自动前置，帧号/自计时时间戳 §7.1）+ 可选眼点 → flushUdp。
        void update(const aerovista::sync::cigi_wire::EyePose* eye);

        /// 从 entities.json 子集建表（不发送）。
        bool loadEntityCatalog(const std::string& path, std::string* error = nullptr);
        std::vector<aerovista::sync::EntityAuthorityRow> entitySnapshot() const;
        std::optional<aerovista::sync::EntityAuthorityRow> entityRow(std::uint16_t entityId) const;

        /// 只写表：EntityCtrl 上的字段一次写齐（不组包、不 flush）。未知 id 失败。
        bool setEntityCtrl(std::uint16_t entityId, aerovista::sync::EntityAuthorityState state, std::uint8_t alpha,
                           std::string* error = nullptr);
        /// 只写表：EntityPositionCtrl 上的 last pose（不组包、不 flush）。未知 id 失败。
        bool setEntityPose(std::uint16_t entityId, const aerovista::sync::EntityAuthorityPose& pose,
                           std::string* error = nullptr);

        /// 按当前表组请求的报文族，一次 flushTcp。未请求的报文族不发。未知 id 失败且不 flush。
        struct EntitySend
        {
            bool entityCtrl = false;
            bool entityPosition = false;
        };
        bool sendEntity(std::uint16_t entityId, EntitySend send, std::string* error = nullptr);

        /// 报文自检：随机构造一个命令面（TCP）测试报文并发送，返回报文类名。
        /// 配合 engine HUD「recv: <类名>」对照验证各报文链路支持（cigi梳理.md 链路矩阵）。
        std::string sendRandomTcpPacket();
        /// 报文自检：随机构造一个数据面（UDP）测试报文并发送，返回报文类名。
        std::string sendRandomUdpPacket();

        /// 接收轮询：drain IG→Host 收包队列并解包，触发订阅回调（Host push 模式，UI 定时器每帧调用）。
        void pollIncoming();
        /// 注册某类 IG→Host 报文的到达回调（转发 HostSync::addCallback，状态同步设计初版.md §8.1）。
        /// 回调在 pollIncoming（UI 线程）同步调用；本示例只置位报文名。
        /// 解包栈内勿做对话框重绘 / 磁盘 IO。
        template <typename PacketT>
        void addCallback(std::function<void(const PacketT&)> callback)
        {
            _host.addCallback<PacketT>(std::move(callback));
        }

        bool isRunning() const;
        int readyIgCount() const;
        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

    private:
        aerovista::sync::HostSync _host;
        aerovista::sync::HostDataManager _data;
        bool _initialized = false;
    };
} // namespace aerovista::viewhost
