#pragma once

#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <functional>
#include <string>

namespace aerovista::viewhost
{
    /// HostSync 薄封装：生命周期 + 帧驱动 + 状态读取（viewhost设计.md §4）。
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

        /// 命令面（TCP）一次性摆放实体位姿（Detach+LLA，EntityID≠0）→ flushTcp。
        /// Host 控制 IG 侧实体位姿（engine `updateEntityPose` 订阅消费，状态同步设计初版.md §12）。
        void sendEntityPose(std::uint16_t entityId, double lat, double lon, double alt, double yawDeg,
                            double pitchDeg, double rollDeg);

        /// 报文自检：随机构造一个命令面（TCP）测试报文并发送，返回报文类名。
        /// 配合 engine HUD「recv: <类名>」对照验证各报文链路支持（cigi梳理.md 链路矩阵）。
        std::string sendRandomTcpPacket();
        /// 报文自检：随机构造一个数据面（UDP）测试报文并发送，返回报文类名。
        std::string sendRandomUdpPacket();

        /// 接收轮询：drain IG→Host 收包队列并解包，触发订阅回调（Host push 模式，UI 定时器每帧调用）。
        void pollIncoming();
        /// 订阅某类 IG→Host 报文的到达通知（转发 HostSync::subscribe，§8.1）。
        /// 回调在 pollIncoming 内同步调用（UI 线程），只做轻量置位/入队。
        template <typename PacketT>
        void subscribe(std::function<void(const PacketT&)> callback)
        {
            _host.subscribe<PacketT>(std::move(callback));
        }

        bool isRunning() const;
        int readyIgCount() const;
        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

    private:
        aerovista::sync::HostSync _host;
        bool _initialized = false;
    };
} // namespace aerovista::viewhost
