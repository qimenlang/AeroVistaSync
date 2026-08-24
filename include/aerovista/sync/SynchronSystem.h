#pragma once

#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SyncMath.h>

#include <cstdint>
#include <memory>
#include <optional>

namespace aerovista::sync
{
    /// IG 位姿决策器（preFrame / update 循环）。
    /// 持有 IgSync，收包后做 frame 校验 / offset 合成 / stale policy / 断线兜底，
    /// 经 takePendingCameraPose() 产出本帧应写相机的位姿。见 doc/design/多通道同步模块设计.md。
    ///
    /// 数据流契约（sync模块化设计.md §3.1）：Host 眼点由宿主经 queueHostEyePose()
    /// 喂入（测试注入）或经 preFrame() 从 IgSync 收包，场景模式由宿主注入
    /// setEllipsoidTransform()，产出位姿由宿主经 takePendingCameraPose() 取走。
    ///
    /// 公开接口零 vsg：所有类型为自有 POD（DVec3）或注入接口（EllipsoidTransform），
    /// 消费方（含无 vsg 的 viewhost）不接触 vsg 头文件。Host 采样/扇出不在本类，
    /// 由宿主（Engine）自行持有 HostSync 完成。
    class SynchronSystem
    {
    public:
        SynchronSystem();
        ~SynchronSystem();

        static std::unique_ptr<SynchronSystem> create();

        /// 初始化 IG 决策器：按 role 启动 IgSync，并应用装配配置
        /// （channelId / offsetDeg / hostEyeStalePolicy / requireConnectedIg）。
        bool initialize(const SyncRoleConfig& role, const SyncSystemConfig& syncSystem);
        void shutdown();

        /// 帧前：IgSync::drainIncoming 收包解包 + update 帧级维护，取眼点入队（本帧相机决策的输入）。
        void preFrame();

        /// 场景模式注入（lla §2 / §4.5）：宿主场景确定或重建后调用。
        /// `transform` 非空 = 椭球模式；空 = 本地模式（唯一场景模式入口）。
        /// sync 库不持有所有权，宿主需保证其生命周期覆盖 sync 会话。
        void setEllipsoidTransform(const EllipsoidTransform* transform);
        /// 通道标识（错误日志用）。
        void setChannelId(int channelId);

        /// 帧级决策（不收包）：计算本帧待应用相机位姿（若有）。
        /// 收包在 preFrame（IgSync::drainIncoming）；每帧调用一次，然后读 takePendingCameraPose()。
        void update();

        /// 本帧应写相机的位姿（Host 眼点 ⊕ 偏移；断线 / ReuseLast 时保留最后一帧），若有。
        /// 由调用方取走（取走即清空）并驱动相机：WorldLocal → setCameraPose，LLA → setCameraPoseLla。
        std::optional<HostEyePose> takePendingCameraPose();

        bool hasIg() const { return static_cast<bool>(_ig); }

        IgSync& igSync();

        void setOffsetDeg(const OffsetDeg& offset);
        const OffsetDeg& offsetDeg() const { return _offsetDeg; }

        /// 把通道偏移合成到 Host 眼点上（刚性阵列旋转）
        /// R_ig = R_host · R_offset（Host 有 roll 时保持各通道 up 轴平行；lla设计 §3.4）。
        static HostEyePose compose(const HostEyePose& host, const OffsetDeg& offset);

        void setHostEyeStalePolicy(HostEyeStalePolicy policy);
        HostEyeStalePolicy hostEyeStalePolicy() const { return _stalePolicy; }

        /// 测试 / 注入：入队一个 Host 眼点（如同本帧随 IGCtrl 收到）。
        void queueHostEyePose(const HostEyePose& pose);

        /// IG TCP+UDP 均就绪。
        bool igLinked() const;

        /// update 最近写入的位姿（Host ⊕ 偏移），若有。
        std::optional<HostEyePose> lastAppliedHostEye() const { return _lastApplied; }

        /// 因线上帧类型（Attach/Detach）≠ 本地场景模式而丢弃的 Host 眼点数（lla §4.5）。
        std::uint64_t eyePoseRejectedByFrameMismatch() const { return _eyePoseRejectedByFrameMismatch; }

        /// 图形重建 / 模式切换后清空眼点缓存（lla §4.3）；不拆除网络。
        void resetEyeCaches();

    private:
        void applyHostEye(const HostEyePose& hostEye);
        bool tryAcceptPendingEye();
        /// 椭球变换存在即椭球模式（本地 = 无椭球变换）。
        bool sceneIsEllipsoid() const { return _ellipsoidTransform != nullptr; }

        std::unique_ptr<IgSync> _ig;

        OffsetDeg _offsetDeg{};
        HostEyeStalePolicy _stalePolicy = HostEyeStalePolicy::REUSE_LAST;

        const EllipsoidTransform* _ellipsoidTransform = nullptr;
        int _channelId = 0;

        bool _hasPendingEye = false;
        HostEyePose _pendingEye{};
        std::optional<HostEyePose> _cachedHostEye;
        std::optional<HostEyePose> _lastApplied;
        std::optional<HostEyePose> _pendingApplied;
        std::uint64_t _eyePoseRejectedByFrameMismatch = 0;
        bool _frameMismatchErrorLogged = false;
    };
} // namespace aerovista::sync
