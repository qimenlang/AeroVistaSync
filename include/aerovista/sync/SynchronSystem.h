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
    /// Selects CIGI Attach (WORLD_LOCAL) vs Detach (LLA). See lla位姿传输设计.md §5.
    enum class HostEyeCoordFrame : std::uint8_t
    {
        WORLD_LOCAL = 0,
        LLA = 1
    };

    /// Host eye (pos + Euler YPR degrees). `frame` drives wire Attach/Detach (not a private UDP field).
    struct HostEyePose
    {
        vsg::dvec3 position{}; ///< WORLD_LOCAL: XYZ m; LLA: lat°, lon°, alt m
        vsg::dvec3 eulerYprDeg{};
        HostEyeCoordFrame frame = HostEyeCoordFrame::WORLD_LOCAL;
    };

    /// Engine-facing sync facade (loop preFrame / update / postFrame).
    /// Owns IgSync and optionally HostSync. See doc/design/多通道同步模块设计.md.
    ///
    /// Data-flow contract (sync模块化设计.md §7): the facade computes the camera pose
    /// each frame; the host engine reads it via takePendingCameraPose() and drives its
    /// own camera. Scene mode / ellipsoid / channel are injected by the host, and the
    /// authority LookAt is fed via captureAuthorityEye() — the facade never touches the
    /// engine's camera directly.
    class SynchronSystem : public vsg::Inherit<vsg::Object, SynchronSystem>
    {
    public:
        SynchronSystem();
        ~SynchronSystem() override;

        /// If requireIgConnect is false, IgSync is initialized locally even when connect fails.
        bool initialize(const SyncRoleConfig& role, bool requireIgConnect = true);
        void shutdown();

        void preFrame();

        /// Scene mode + ellipsoid injection (lla §2 / §4.5): call when the host scene is
        /// known or rebuilt. `in_ellipsoid` may be null in Local mode.
        void setSceneIsEllipsoid(bool sceneIsEllipsoid);
        void setEllipsoidModel(vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid);
        /// Channel identity for diagnostics (error logs).
        void setChannelId(int channelId);

        /// Sample the authority eye after handleEvents (Host engines only): feed the
        /// current camera LookAt (pre-overwrite). The facade decides LLA vs WorldLocal
        /// from the injected scene mode and applies the anti-echo check.
        void captureAuthorityEye(const vsg::LookAt& lookAt);

        /// Advance sync state (recv / decision); computes this frame's pending camera
        /// pose if any. Call once per frame, then read takePendingCameraPose().
        void update();

        /// This frame's camera pose to apply (Host eye ⊕ offset; keep-last on disconnect /
        /// ReuseLast), if any. Consumed (cleared) by the caller, which drives its camera:
        /// WorldLocal → setCameraPose, LLA → setCameraPoseLla.
        std::optional<HostEyePose> takePendingCameraPose();

        void postFrame(double simTimeMs);

        bool hasHost() const { return static_cast<bool>(_host); }
        bool hasIg() const { return static_cast<bool>(_ig); }

        HostSync& hostSync();
        IgSync& igSync();

        void setOffsetDeg(const OffsetDeg& offset);
        const OffsetDeg& offsetDeg() const { return _offsetDeg; }

        /// Compose channel offset onto Host eye as a rigid-array rotation
        /// R_ig = R_host · R_offset (keeps up axes parallel under Host roll; lla设计 §3.4).
        static HostEyePose compose(const HostEyePose& host, const OffsetDeg& offset);

        void setHostEyeStalePolicy(HostEyeStalePolicy policy);
        HostEyeStalePolicy hostEyeStalePolicy() const { return _stalePolicy; }

        /// Test / injection: enqueue a Host eye as if received this frame (with IGCtrl).
        void queueHostEyePose(const HostEyePose& pose);

        /// Test / injection: seed `_lastSent` for mode-switch type-discard ATTD (lla §4.3 / §7).
        void seedLastSentHostEye(const HostEyePose& pose);

        /// IG TCP+UDP both ready.
        bool igLinked() const;

        /// Last pose written by update (Host ⊕ offset), if any.
        std::optional<HostEyePose> lastAppliedHostEye() const { return _lastApplied; }

        /// Last authority eye Host packed for fan-out this session (for anti-echo BDD).
        std::optional<HostEyePose> lastSentHostEye() const { return _lastSent; }

        /// Count of Host eyes dropped because wire frame (Attach/Detach) ≠ local scene mode (lla §4.5).
        std::uint64_t eyePoseRejectedByFrameMismatch() const { return _eyePoseRejectedByFrameMismatch; }

        /// Clear eye caches after graphics rebuild / mode switch (lla §4.3); does not tear down network.
        void resetEyeCaches();

    private:
        void applyHostEye(const HostEyePose& hostEye);
        bool tryAcceptPendingEye();

        SyncRoleConfig _role{};
        std::unique_ptr<HostSync> _host;
        std::unique_ptr<IgSync> _ig;

        OffsetDeg _offsetDeg{};
        HostEyeStalePolicy _stalePolicy = HostEyeStalePolicy::REUSE_LAST;

        bool _sceneIsEllipsoid = false;
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
