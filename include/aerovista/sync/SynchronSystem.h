#pragma once

#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <memory>
#include <optional>

namespace aerovista::sync
{
    /// IG 侧同步收发端点：持有 IgSync，负责收包解包 + 帧级维护 + 连接状态查询。
    /// 不承担眼点业务。本仓库里：offset 合成在 Engine `CameraDriver`
    /// （`Engine::registerIgCallbacks` → `CameraDriver::onOwnshipEyePose`）；写相机在
    /// `Engine::applyLastHostEye`。公开接口零 vsg。Host 采样/扇出由 viewhost `HostDriver` 完成。
    class SynchronSystem
    {
    public:
        // ===== 对外业务面（消费方：engine）=====

        SynchronSystem();
        ~SynchronSystem();

        static std::unique_ptr<SynchronSystem> create();

        // ---- 生命周期 ----
        /// 初始化 IG 收发端点：`igConfig` 非空则启动 IgSync 并连接，空则不启 IG。
        /// `requireConnectedIg` 控制 connect 失败是否拒绝；`channelId` 仅存储（当前无运行期消费）；
        /// `offsetDeg` 由 Engine `CameraDriver` 消费，本类不读。眼点订阅由 Engine 在本函数返回后注册。
        bool initialize(const std::optional<IgConfig>& igConfig, const SyncSystemConfig& syncSystem);
        void shutdown();

        // ---- 帧循环（engine tickOnFrame 每帧驱动）----
        /// 帧前：`IgSync::drainIncoming` 收包解包 + `IgSync::update` 帧级维护。
        /// 解包时业务回调同步翻译/合成眼点；返回后由 `Engine::applyLastHostEye` 写相机。
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
