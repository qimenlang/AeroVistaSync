#pragma once

#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
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

        bool isRunning() const;
        int readyIgCount() const;
        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

    private:
        aerovista::sync::HostSync _host;
        bool _initialized = false;
    };
} // namespace aerovista::viewhost
