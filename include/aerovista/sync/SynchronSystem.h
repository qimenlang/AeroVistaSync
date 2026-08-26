#pragma once

#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <cstdint>
#include <memory>
#include <optional>

namespace aerovista::sync
{
    /// IG 位姿决策器（preFrame / update 循环）。
    /// 持有 IgSync，收包后做 frame 校验 / offset 合成 / stale policy / 断线兜底，
    /// 经 takePendingCameraPose() 产出本帧应写相机的位姿。见 doc/design/多通道同步模块设计.md。
    ///
    /// 数据流契约（sync模块化设计.md §3.1）：Host 眼点经 subscribeEyePose 订阅投递（或测试
    /// queueHostEyePose 注入）入队决策器，场景模式由宿主 setEllipsoidMode() 注入，产出位姿经
    /// takePendingCameraPose() 取走。
    ///
    /// 公开接口零 vsg：所有类型为自有 POD（DVec3）。消费方（含无 vsg 的 viewhost）
    /// 不接触 vsg 头文件。Host 采样/扇出不在本类，由宿主（Engine）自行持有 HostSync 完成。
    class SynchronSystem
    {
    public:
        // ===== 对外业务面（消费方：engine）=====

        SynchronSystem();
        ~SynchronSystem();

        static std::unique_ptr<SynchronSystem> create();

        // ---- 生命周期 ----
        /// 初始化 IG 决策器：`igConfig` 非空则按它启动 IgSync 并连接，空则不启 IG（纯 Host 宿主 / 关闭同步）；
        /// 装配配置（channelId / offsetDeg / hostEyeStalePolicy / requireConnectedIg）经 `syncSystem` 应用。
        bool initialize(const std::optional<IgConfig>& igConfig, const SyncSystemConfig& syncSystem);
        void shutdown();

        // ---- 帧循环（engine tickOnFrame 每帧驱动）----
        /// 帧前：IgSync::drainIncoming 收包解包 + update 帧级维护，取眼点入队（本帧相机决策的输入）。
        void preFrame();
        /// 帧级决策（不收包）：计算本帧待应用相机位姿（若有）。
        /// 收包在 preFrame（IgSync::drainIncoming）；每帧调用一次，然后读 takePendingCameraPose()。
        void update();
        /// 本帧应写相机的位姿（Host 眼点 ⊕ 偏移；断线 / ReuseLast 时保留最后一帧），若有。
        /// 由调用方取走（取走即清空）并驱动相机：WorldLocal → setCameraPose，LLA → setCameraPoseLla。
        std::optional<HostEyePose> takePendingCameraPose();

        // ---- 场景模式 / 装配 ----
        /// 场景模式注入（lla §2 / §4.5）：`ellipsoid` 为 true = 椭球模式，false = 本地模式（唯一入口）。
        /// 由宿主在场景确定或重建后调用；引擎侧取场景 `EllipsoidModel` 有无即可。
        void setEllipsoidMode(bool ellipsoid);
        void setOffsetDeg(const OffsetDeg& offset);
        const OffsetDeg& offsetDeg() const { return _offsetDeg; }
        void setHostEyeStalePolicy(HostEyeStalePolicy policy);
        HostEyeStalePolicy hostEyeStalePolicy() const { return _stalePolicy; }

        // ---- 位姿合成工具 ----
        /// 把通道偏移合成到 Host 眼点上（刚性阵列旋转）
        /// R_ig = R_host · R_offset（Host 有 roll 时保持各通道 up 轴平行；lla设计 §3.4）。
        static HostEyePose compose(const HostEyePose& host, const OffsetDeg& offset);

        // ---- 状态观测 / 运维 ----
        bool hasIg() const { return static_cast<bool>(_ig); }
        /// IG TCP+UDP 均就绪。
        bool igLinked() const;
        /// update 最近写入的位姿（Host ⊕ 偏移），若有。
        std::optional<HostEyePose> lastAppliedHostEye() const { return _lastApplied; }
        /// 因线上帧类型（Attach/Detach）≠ 本地场景模式而丢弃的 Host 眼点数（lla §4.5）。
        std::uint64_t eyePoseRejectedByFrameMismatch() const { return _eyePoseRejectedByFrameMismatch; }
        /// 图形重建 / 模式切换后清空眼点缓存（lla §4.3）；不拆除网络。
        void resetEyeCaches();

        // ---- 内部组件访问 ----
        IgSync& igSync();

        // ===== 测试注入辅助（当前仅测试消费）=====

        /// 测试 / 注入：入队一个 Host 眼点（如同本帧随 IGCtrl 收到）。
        void queueHostEyePose(const HostEyePose& pose);

    private:
        /// compose + 标记本帧待应用（供 update 帧决策调用；takePendingCameraPose 取走）。
        void applyComposed(const HostEyePose& hostEye);
        /// 本帧新输入 → 帧校验（scene 模式匹配）→ 通过则入缓存并弃输入，失败则拒绝（计数 + 首错日志）。
        bool tryAcceptPendingEye();
        /// frame 校验失败：计数 + 首次错误日志（从 tryAcceptPendingEye 拆出，单一职责）。
        void rejectPendingFrameMismatch();
        bool sceneIsEllipsoid() const { return _ellipsoidMode; }

        std::unique_ptr<IgSync> _ig;

        OffsetDeg _offsetDeg{};
        HostEyeStalePolicy _stalePolicy = HostEyeStalePolicy::REUSE_LAST;

        bool _ellipsoidMode = false;
        int _channelId = 0;

        /// 本帧新输入（收包/注入）；空 = 本帧无新输入。经 tryAcceptPendingEye 消费（成功入缓存或弃值）。
        std::optional<HostEyePose> _pendingHostEye;
        std::optional<HostEyePose> _cachedHostEye;
        /// 最近合成位姿（Host ⊕ offset，持久；供 lastAppliedHostEye 观测）。
        std::optional<HostEyePose> _lastApplied;
        /// 本帧已合成新位姿的事件标记：takePendingCameraPose 读取 `_lastApplied` 并清标记。
        bool _hasPendingApplied = false;
        std::uint64_t _eyePoseRejectedByFrameMismatch = 0;
        bool _frameMismatchErrorLogged = false;
    };
} // namespace aerovista::sync
