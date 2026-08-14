#pragma once

#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <vsg/all.h>

namespace aerovista::sync
{
    /// 选择 CIGI Attach（WORLD_LOCAL）还是 Detach（LLA）。见 lla位姿传输设计.md §5。
    enum class HostEyeCoordFrame : std::uint8_t
    {
        WORLD_LOCAL = 0,
        LLA = 1
    };

    /// Host 眼点（位置 + 欧拉 YPR 度）。`frame` 驱动线上的 Attach/Detach（不是私有 UDP 字段）。
    struct HostEyePose
    {
        vsg::dvec3 position{}; ///< WORLD_LOCAL: XYZ 米；LLA: 纬度°、经度°、海拔 米
        vsg::dvec3 eulerYprDeg{};
        HostEyeCoordFrame frame = HostEyeCoordFrame::WORLD_LOCAL;
    };

    /// 引擎侧同步门面（preFrame / update / postFrame 循环）。
    /// 持有 IgSync，可选持有 HostSync。见 doc/design/多通道同步模块设计.md。
    ///
    /// 数据流契约（sync模块化设计.md §3.1）：门面每帧计算相机位姿，
    /// 宿主引擎经 takePendingCameraPose() 取走并驱动自己的相机。
    /// 场景模式 / 椭球 / 通道号由宿主注入，权威 LookAt 经 captureAuthorityEye() 喂入——
    /// 门面从不直接触碰宿主的相机对象。
    class SynchronSystem : public vsg::Inherit<vsg::Object, SynchronSystem>
    {
    public:
        SynchronSystem();
        ~SynchronSystem() override;

        /// 初始化同步：启动 HostSync / IgSync（按 role），并应用装配配置
        /// （channelId / offsetDeg / hostEyeStalePolicy / requireIgConnect）。
        bool initialize(const SyncRoleConfig& role, const SyncSystemConfig& syncSystem);
        void shutdown();

        void preFrame();

        /// 场景模式注入（lla §2 / §4.5）：宿主场景确定或重建后调用。
        /// `ellipsoid` 非空 = 椭球模式；空 = 本地模式（唯一场景模式入口）。
        void setEllipsoidModel(vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);
        /// 通道标识（错误日志用）。
        void setChannelId(int channelId);

        /// handleEvents 后采样权威眼点（仅 Host 引擎）：喂入当前相机 LookAt（覆盖前）。
        /// 门面根据注入的场景模式决定 LLA 还是 WorldLocal，并做防回声检查。
        void captureAuthorityEye(const vsg::LookAt& lookAt);

        /// 推进同步状态（收包 / 决策）；计算本帧待应用相机位姿（若有）。
        /// 每帧调用一次，然后读 takePendingCameraPose()。
        void update();

        /// 本帧应写相机的位姿（Host 眼点 ⊕ 偏移；断线 / ReuseLast 时保留最后一帧），若有。
        /// 由调用方取走（取走即清空）并驱动相机：WorldLocal → setCameraPose，LLA → setCameraPoseLla。
        std::optional<HostEyePose> takePendingCameraPose();

        void postFrame(double simTimeMs);

        bool hasHost() const { return static_cast<bool>(_host); }
        bool hasIg() const { return static_cast<bool>(_ig); }

        HostSync& hostSync();
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

        /// 测试 / 注入：为模式切换的类型丢弃 ATTD 种子 `_lastSent`（lla §4.3 / §7）。
        void seedLastSentHostEye(const HostEyePose& pose);

        /// IG TCP+UDP 均就绪。
        bool igLinked() const;

        /// update 最近写入的位姿（Host ⊕ 偏移），若有。
        std::optional<HostEyePose> lastAppliedHostEye() const { return _lastApplied; }

        /// 本会话 Host 最近为扇出打包的权威眼点（防回声 BDD 用）。
        std::optional<HostEyePose> lastSentHostEye() const { return _lastSent; }

        /// 因线上帧类型（Attach/Detach）≠ 本地场景模式而丢弃的 Host 眼点数（lla §4.5）。
        std::uint64_t eyePoseRejectedByFrameMismatch() const { return _eyePoseRejectedByFrameMismatch; }

        /// 图形重建 / 模式切换后清空眼点缓存（lla §4.3）；不拆除网络。
        void resetEyeCaches();

    private:
        void applyHostEye(const HostEyePose& hostEye);
        bool tryAcceptPendingEye();
        /// 椭球模型存在即椭球模式（本地 = 无椭球模型）。
        bool sceneIsEllipsoid() const { return static_cast<bool>(_ellipsoidModel); }

        SyncRoleConfig _role{};
        std::unique_ptr<HostSync> _host;
        std::unique_ptr<IgSync> _ig;

        OffsetDeg _offsetDeg{};
        HostEyeStalePolicy _stalePolicy = HostEyeStalePolicy::REUSE_LAST;

        vsg::ref_ptr<vsg::EllipsoidModel> _ellipsoidModel;
        int _channelId = 0;

        bool _hasPendingEye = false;
        HostEyePose _pendingEye{};
        std::optional<HostEyePose> _cachedHostEye;
        std::optional<HostEyePose> _lastApplied;
        std::optional<HostEyePose> _lastSent;
        std::optional<HostEyePose> _frameSample;
        std::optional<HostEyePose> _pendingApplied;
        std::uint64_t _eyePoseRejectedByFrameMismatch = 0;
        bool _frameMismatchErrorLogged = false;
    };
} // namespace aerovista::sync
