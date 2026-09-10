#pragma once

#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <memory>
#include <optional>

namespace aerovista::sync
{
    /// IG 侧同步收发端点：持有 IgSync，负责收包解包 + 帧级维护 + 连接状态查询。
    ///
    /// 职责边界（2026-09 重构）：本类只做「数据收发 + 同步状态」，不承担眼点业务。
    /// Host 眼点的 offset 合成在 Engine `CameraDriver`（`engine/source/function/driver/`）。
    /// 本类经 preFrame 收包后，业务回调（Engine::registerIgCallbacks 转发到
    /// CameraDriver::onOwnshipEyePose）完成眼点翻译。公开接口零 vsg。Host 采样/扇出不在本类，
    /// 由独立 viewhost 进程（HostDriver）完成。
    class SynchronSystem
    {
    public:
        // ===== 对外业务面（消费方：engine）=====

        SynchronSystem();
        ~SynchronSystem();

        static std::unique_ptr<SynchronSystem> create();

        // ---- 生命周期 ----
        /// 初始化 IG 收发端点：`igConfig` 非空则按它启动 IgSync 并连接，空则不启 IG（关闭同步）；
        /// `syncSystem` 提供 channelId / requireConnectedIg（offsetDeg 已上移 Engine CameraDriver，
        /// 本类不再消费）。igConfig 非空才启 IG。
        bool initialize(const std::optional<IgConfig>& igConfig, const SyncSystemConfig& syncSystem);
        void shutdown();

        // ---- 帧循环（engine tickOnFrame 每帧驱动）----
        /// 帧前收包：IgSync::drainIncoming 收包解包 + IgSync::update 帧级维护。
        /// 眼点原始报文经 UDP 通用捕获多播投递，由业务回调（Engine::registerIgCallbacks 转发到
        /// CameraDriver::onOwnshipEyePose）翻译；本类不再做眼点决策。每帧调用一次，随后由
        /// Engine::applyLastHostEye 写相机。
        void preFrame();

        // ---- 状态观测 / 运维 ----
        bool hasIg() const { return static_cast<bool>(_ig); }
        /// IG TCP+UDP 均就绪（连接观测；CameraDriver 不读此项）。
        bool igLinked() const;

        // ---- 内部组件访问 ----
        IgSync& igSync();

    private:
        std::unique_ptr<IgSync> _ig;

        int _channelId = 0;
    };
} // namespace aerovista::sync
