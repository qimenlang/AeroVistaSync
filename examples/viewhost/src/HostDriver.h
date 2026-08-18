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

        /// 扇出一帧 IGCtrl（可选眼点）。
        void update(double simTimeMs, const aerovista::sync::HostSync::EyePose* eye);

        bool isRunning() const;
        int readyIgCount() const;
        std::uint32_t igCtrlSentCount() const;
        std::uint32_t sofReceivedCount() const;

    private:
        aerovista::sync::HostSync _host;
        bool _initialized = false;
    };
} // namespace aerovista::viewhost
