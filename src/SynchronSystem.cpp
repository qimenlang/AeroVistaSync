#include <aerovista/sync/SynchronSystem.h>

#include <vsg/maths/common.h>
#include <vsg/maths/quat.h>
#include <vsg/maths/vec3.h>

#include <cmath>
#include <iostream>
#include <utility>

namespace aerovista::sync
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        /// 公开边界 DVec3 ↔ 内部 vsg::dvec3（仅 .cpp 内转换，不泄漏 vsg 类型到公开头）。
        vsg::dvec3 toVsg(const DVec3& v)
        {
            return vsg::dvec3(v.x, v.y, v.z);
        }

        DVec3 toSync(const vsg::dvec3& v)
        {
            return DVec3{v.x, v.y, v.z};
        }

        double rad2deg(double r)
        {
            return r * (180.0 / kPi);
        }

        double clampd(double v, double lo, double hi)
        {
            return v < lo ? lo : (v > hi ? hi : v);
        }

        /// Engine::setCameraPose 旋转 Rz(yaw)*Rx(pitch)*Ry(roll) 的逆，Y-forward Z-up。
        /// 从正交 forward/up 基恢复 YPR（度）。
        bool extractYprDegFromBasis(const vsg::dvec3& forward, const vsg::dvec3& up, vsg::dvec3& eulerYprDegOut)
        {
            const double yawRad = std::atan2(-forward.x, forward.y);
            const double pitchRad = std::asin(clampd(forward.z, -1.0, 1.0));

            // yaw+pitch 足够：先 pitch 后 yaw（VSG reverse-Hamilton）。
            const vsg::dvec3 afterPitchUp =
                vsg::dquat(pitchRad, vsg::dvec3(1.0, 0.0, 0.0)) * vsg::dvec3(0.0, 0.0, 1.0);
            const vsg::dvec3 afterPitchRight =
                vsg::dquat(pitchRad, vsg::dvec3(1.0, 0.0, 0.0)) * vsg::dvec3(1.0, 0.0, 0.0);
            const vsg::dvec3 expectedUp =
                vsg::normalize(vsg::dquat(yawRad, vsg::dvec3(0.0, 0.0, 1.0)) * afterPitchUp);
            const vsg::dvec3 expectedRight =
                vsg::normalize(vsg::dquat(yawRad, vsg::dvec3(0.0, 0.0, 1.0)) * afterPitchRight);
            const double rollRad = std::atan2(vsg::dot(up, expectedRight), vsg::dot(up, expectedUp));

            eulerYprDegOut = vsg::dvec3(rad2deg(yawRad), rad2deg(pitchRad), rad2deg(rollRad));
            return true;
        }

        bool eyeFrameMatchesScene(const HostEyePose& eye, bool sceneIsEllipsoid)
        {
            const bool wantLla = (eye.frame == HostEyeCoordFrame::LLA);
            return wantLla == sceneIsEllipsoid;
        }
    } // namespace

    std::unique_ptr<SynchronSystem> SynchronSystem::create()
    {
        return std::make_unique<SynchronSystem>();
    }

    SynchronSystem::SynchronSystem() = default;

    SynchronSystem::~SynchronSystem()
    {
        shutdown();
    }

    bool SynchronSystem::initialize(const SyncRoleConfig& role, const SyncSystemConfig& syncSystem)
    {
        shutdown();
        _channelId = syncSystem.channelId;
        _offsetDeg = syncSystem.offsetDeg;
        _stalePolicy = syncSystem.hostEyeStalePolicy;

        if (role.enableIg)
        {
            _ig = std::make_unique<IgSync>();
            if (!_ig->initialize(role.igConfig))
            {
                std::cerr << "SynchronSystem: IgSync initialize failed\n";
                shutdown();
                return false;
            }

            if (!_ig->connect(role.igConfig))
            {
                if (syncSystem.requireConnectedIg)
                {
                    std::cerr << "SynchronSystem: IgSync connect failed\n";
                    shutdown();
                    return false;
                }
            }
        }

        return true;
    }

    void SynchronSystem::shutdown()
    {
        if (_ig)
        {
            _ig->shutdown();
            _ig.reset();
        }
        resetEyeCaches();
    }

    void SynchronSystem::resetEyeCaches()
    {
        _hasPendingEye = false;
        _pendingEye = {};
        _cachedHostEye.reset();
        _lastApplied.reset();
        _frameMismatchErrorLogged = false;
    }

    void SynchronSystem::preFrame()
    {
        if (!_ig)
            return;

        _ig->update(/*sendSof=*/true);
        if (auto eye = _ig->takeReceivedHostEye())
        {
            HostEyePose pose;
            pose.position = DVec3{eye->x, eye->y, eye->z};
            pose.eulerYprDeg = DVec3{eye->yawDeg, eye->pitchDeg, eye->rollDeg};
            pose.frame = eye->isLla ? HostEyeCoordFrame::LLA : HostEyeCoordFrame::WORLD_LOCAL;
            queueHostEyePose(pose);
        }
    }

    HostEyePose SynchronSystem::compose(const HostEyePose& host, const OffsetDeg& offset)
    {
        // 刚性阵列通道偏移：R_ig = R_host · R_offset（Hamilton）。对纯 yaw 偏移，
        // 绕 Host 自身 up 轴旋转 Host 的 forward，所以每个通道的 up 与 Host 保持平行——
        // 边缘对边缘的 frustum 拼接在 Host roll 下仍成立。若用分量式 YPR 相加，得到
        // Rz(δ)·R_host，一旦 roll ≠ 0 各通道 up 轴就分开（roll 撕裂 bug；lla设计 §3.4）。
        //
        // VSG operator*(a,b) = Hamilton(b⊗a)，所以写 qOffset * qHost 得到
        // M(qHost)·M(qOffset) = R_host·R_offset。每个四元数按 Ry(roll)*Rx(pitch)*Rz(yaw)
        // （VSG）构建，以表示 Hamilton 的 Rz(yaw)*Rx(pitch)*Ry(roll) 写约定。
        const vsg::dvec3 hostEuler = toVsg(host.eulerYprDeg);
        const vsg::dquat qHost =
            vsg::dquat(vsg::radians(hostEuler.z), vsg::dvec3(0.0, 1.0, 0.0)) *
            vsg::dquat(vsg::radians(hostEuler.y), vsg::dvec3(1.0, 0.0, 0.0)) *
            vsg::dquat(vsg::radians(hostEuler.x), vsg::dvec3(0.0, 0.0, 1.0));
        const vsg::dquat qOffset =
            vsg::dquat(vsg::radians(offset.roll), vsg::dvec3(0.0, 1.0, 0.0)) *
            vsg::dquat(vsg::radians(offset.pitch), vsg::dvec3(1.0, 0.0, 0.0)) *
            vsg::dquat(vsg::radians(offset.yaw), vsg::dvec3(0.0, 0.0, 1.0));
        const vsg::dquat qIg = qOffset * qHost;

        const vsg::dvec3 forward = vsg::normalize(qIg * vsg::dvec3(0.0, 1.0, 0.0));
        const vsg::dvec3 up = vsg::normalize(qIg * vsg::dvec3(0.0, 0.0, 1.0));

        // 在同一 Rz·Rx·Ry 约定下重新提取 YPR，使 applyHostEye 的 setCameraPose
        // 精确写入合成后的旋转（写↔采样保持互逆）。
        HostEyePose out = host;
        vsg::dvec3 eulerYprDeg;
        extractYprDegFromBasis(forward, up, eulerYprDeg);
        out.eulerYprDeg = toSync(eulerYprDeg);
        return out;
    }

    bool SynchronSystem::tryAcceptPendingEye()
    {
        if (!_hasPendingEye)
            return false;

        if (!eyeFrameMatchesScene(_pendingEye, sceneIsEllipsoid()))
        {
            ++_eyePoseRejectedByFrameMismatch;
            if (!_frameMismatchErrorLogged)
            {
                const char* expected = sceneIsEllipsoid() ? "Lla/Detach" : "WorldLocal/Attach";
                const char* got = (_pendingEye.frame == HostEyeCoordFrame::LLA) ? "Lla/Detach" : "WorldLocal/Attach";
                std::cerr << "[ERROR] eye pose rejected by frame mismatch: expected " << expected
                          << ", got " << got << " (channelId=" << _channelId << ")\n";
                _frameMismatchErrorLogged = true;
            }
            _hasPendingEye = false;
            return false;
        }

        _cachedHostEye = _pendingEye;
        _hasPendingEye = false;
        return true;
    }

    void SynchronSystem::applyHostEye(const HostEyePose& hostEye)
    {
        const HostEyePose composed = compose(hostEye, _offsetDeg);
        _lastApplied = composed;
        _pendingApplied = composed;
    }

    void SynchronSystem::update()
    {
        // 命令执行归主线程（状态同步设计初版.md §4）：命令读循环线程回 RECEIVED 后入队，
        // 主线程每帧在此执行（场景归属主线程，避免跨线程改场景 / 编译 pipeline）。
        if (_ig)
            _ig->runPendingCommands();

        _pendingApplied.reset();
        const bool linked = igLinked();

        if (linked)
        {
            if (_hasPendingEye)
            {
                if (tryAcceptPendingEye())
                    applyHostEye(*_cachedHostEye);
            }
            else if (_cachedHostEye)
            {
                if (_stalePolicy == HostEyeStalePolicy::REUSE_LAST)
                    applyHostEye(*_cachedHostEye);
                // Freeze：相机保持不动
            }
            return;
        }

        // 未连接：丢弃任何注入的待处理眼点（从未连接不得应用）。
        _hasPendingEye = false;

        // 断线后（或仍持有先前连接的缓存时），保留最后一帧 Host 眼点。
        if (_cachedHostEye)
            applyHostEye(*_cachedHostEye);
    }

    void SynchronSystem::setEllipsoidTransform(const EllipsoidTransform* transform)
    {
        _ellipsoidTransform = transform;
    }

    void SynchronSystem::setChannelId(int channelId)
    {
        _channelId = channelId;
    }

    std::optional<HostEyePose> SynchronSystem::takePendingCameraPose()
    {
        return std::exchange(_pendingApplied, std::nullopt);
    }

    IgSync& SynchronSystem::igSync()
    {
        return *_ig;
    }

    void SynchronSystem::setOffsetDeg(const OffsetDeg& offset)
    {
        _offsetDeg = offset;
    }

    void SynchronSystem::setHostEyeStalePolicy(HostEyeStalePolicy policy)
    {
        _stalePolicy = policy;
    }

    void SynchronSystem::queueHostEyePose(const HostEyePose& pose)
    {
        _pendingEye = pose;
        _hasPendingEye = true;
    }

    bool SynchronSystem::igLinked() const
    {
        return _ig && _ig->tcpConnected() && _ig->udpSynced();
    }
} // namespace aerovista::sync
