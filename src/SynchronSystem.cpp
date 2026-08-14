#include <aerovista/sync/SynchronSystem.h>

#include <cmath>
#include <iostream>
#include <utility>

namespace aerovista::sync
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kLookDistance = 1.0;

        double rad2deg(double r)
        {
            return r * (180.0 / kPi);
        }

        double clampd(double v, double lo, double hi)
        {
            return v < lo ? lo : (v > hi ? hi : v);
        }

        void enuBasisFromLocalToWorld(const vsg::dmat4& localToWorld, vsg::dvec3& east, vsg::dvec3& north, vsg::dvec3& upAxis)
        {
            east = vsg::normalize(vsg::dvec3(localToWorld(0, 0), localToWorld(0, 1), localToWorld(0, 2)));
            north = vsg::normalize(vsg::dvec3(localToWorld(1, 0), localToWorld(1, 1), localToWorld(1, 2)));
            upAxis = vsg::normalize(vsg::dvec3(localToWorld(2, 0), localToWorld(2, 1), localToWorld(2, 2)));
        }

        vsg::dvec3 rotateEnuToEcef(const vsg::dmat4& localToWorld, const vsg::dvec3& enuDir)
        {
            vsg::dvec3 east, north, upAxis;
            enuBasisFromLocalToWorld(localToWorld, east, north, upAxis);
            return enuDir.x * east + enuDir.y * north + enuDir.z * upAxis;
        }

        /// R = Rz(yaw)*Rx(pitch)*Ry(roll)。按 roll→pitch→yaw 顺序作用轴四元数
        /// （VSG 四元数乘法是 reverse-Hamilton）。
        vsg::dvec3 rotateByEulerYprDeg(const vsg::dvec3& eulerYprDeg, const vsg::dvec3& v)
        {
            const vsg::dvec3 afterRoll =
                vsg::dquat(vsg::radians(eulerYprDeg.z), vsg::dvec3(0.0, 1.0, 0.0)) * v;
            const vsg::dvec3 afterPitch =
                vsg::dquat(vsg::radians(eulerYprDeg.y), vsg::dvec3(1.0, 0.0, 0.0)) * afterRoll;
            return vsg::dquat(vsg::radians(eulerYprDeg.x), vsg::dvec3(0.0, 0.0, 1.0)) * afterPitch;
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

        /// Engine::setCameraPose 旋转 Rz(yaw)*Rx(pitch)*Ry(roll) 的逆，Y-forward Z-up。
        bool lookAtToWorldLocalEye(const vsg::LookAt& lookAt, HostEyePose& out)
        {
            out.frame = HostEyeCoordFrame::WORLD_LOCAL;
            out.position = lookAt.eye;
            const vsg::dvec3 forward = vsg::normalize(lookAt.center - lookAt.eye);
            if (vsg::length(forward) < 1e-12)
                return false;
            return extractYprDegFromBasis(forward, vsg::normalize(lookAt.up), out.eulerYprDeg);
        }

        bool lookAtToLlaEye(const vsg::LookAt& lookAt, const vsg::EllipsoidModel& ellipsoid, HostEyePose& out)
        {
            out.frame = HostEyeCoordFrame::LLA;
            out.position = ellipsoid.convertECEFToLatLongAltitude(lookAt.eye);
            const vsg::dvec3 forwardEcef = vsg::normalize(lookAt.center - lookAt.eye);
            if (vsg::length(forwardEcef) < 1e-12)
                return false;

            // 逆写路径（§3.3）：ENU 基 = LocalToWorld 的正交列。
            // 优先用列点积而非 worldToLocal*dir——保持采样是 setCameraPoseLla 的逆。
            const vsg::dmat4 localToWorld = ellipsoid.computeLocalToWorldTransform(out.position);
            const vsg::dvec3 east = vsg::normalize(vsg::dvec3(localToWorld(0, 0), localToWorld(0, 1), localToWorld(0, 2)));
            const vsg::dvec3 north = vsg::normalize(vsg::dvec3(localToWorld(1, 0), localToWorld(1, 1), localToWorld(1, 2)));
            const vsg::dvec3 upAxis = vsg::normalize(vsg::dvec3(localToWorld(2, 0), localToWorld(2, 1), localToWorld(2, 2)));
            const auto toEnu = [&](const vsg::dvec3& ecefDir) {
                return vsg::normalize(vsg::dvec3(vsg::dot(ecefDir, east), vsg::dot(ecefDir, north), vsg::dot(ecefDir, upAxis)));
            };

            const vsg::dvec3 forward = toEnu(forwardEcef);
            const vsg::dvec3 up = toEnu(vsg::normalize(lookAt.up));
            return extractYprDegFromBasis(forward, up, out.eulerYprDeg);
        }

        bool lookAtMatchesApplied(const vsg::LookAt& actual, const HostEyePose& applied,
                                  const vsg::EllipsoidModel* ellipsoid)
        {
            auto lookAt = vsg::LookAt::create();
            if (applied.frame == HostEyeCoordFrame::LLA)
            {
                if (!ellipsoid)
                    return false;
                const vsg::dvec3 forwardEnu = rotateByEulerYprDeg(applied.eulerYprDeg, vsg::dvec3(0.0, 1.0, 0.0));
                const vsg::dvec3 upEnu = rotateByEulerYprDeg(applied.eulerYprDeg, vsg::dvec3(0.0, 0.0, 1.0));
                const vsg::dmat4 localToWorld = ellipsoid->computeLocalToWorldTransform(applied.position);
                const vsg::dvec3 eye = ellipsoid->convertLatLongAltitudeToECEF(applied.position);
                const vsg::dvec3 forward = vsg::normalize(rotateEnuToEcef(localToWorld, forwardEnu));
                const vsg::dvec3 up = vsg::normalize(rotateEnuToEcef(localToWorld, upEnu));
                lookAt->eye = eye;
                lookAt->center = eye + forward * kLookDistance;
                lookAt->up = up;
                constexpr double kEyeEps = 1e-2;
                constexpr double kDirEps = 1e-6;
                const vsg::dvec3 af = vsg::normalize(actual.center - actual.eye);
                const vsg::dvec3 ef = vsg::normalize(lookAt->center - lookAt->eye);
                return vsg::length(actual.eye - lookAt->eye) < kEyeEps &&
                       vsg::length(af - ef) < kDirEps &&
                       vsg::length(vsg::normalize(actual.up) - vsg::normalize(lookAt->up)) < kDirEps;
            }

            const vsg::dvec3 forward = rotateByEulerYprDeg(applied.eulerYprDeg, vsg::dvec3(0.0, 1.0, 0.0));
            const vsg::dvec3 up = rotateByEulerYprDeg(applied.eulerYprDeg, vsg::dvec3(0.0, 0.0, 1.0));
            constexpr double kEps = 1e-4;
            const vsg::dvec3 af = vsg::normalize(actual.center - actual.eye);
            return vsg::length(actual.eye - applied.position) < kEps &&
                   vsg::length(af - vsg::normalize(forward)) < kEps &&
                   vsg::length(vsg::normalize(actual.up) - vsg::normalize(up)) < kEps;
        }

        bool eyeFrameMatchesScene(const HostEyePose& eye, bool sceneIsEllipsoid)
        {
            const bool wantLla = (eye.frame == HostEyeCoordFrame::LLA);
            return wantLla == sceneIsEllipsoid;
        }
    } // namespace

    SynchronSystem::SynchronSystem() = default;

    SynchronSystem::~SynchronSystem()
    {
        shutdown();
    }

    bool SynchronSystem::initialize(const SyncRoleConfig& role, const SyncSystemConfig& syncSystem)
    {
        shutdown();
        _role = role;
        _channelId = syncSystem.channelId;
        _offsetDeg = syncSystem.offsetDeg;
        _stalePolicy = syncSystem.hostEyeStalePolicy;

        if (role.enableHost)
        {
            _host = std::make_unique<HostSync>();
            if (!_host->initialize(role.hostConfig))
            {
                std::cerr << "SynchronSystem: HostSync initialize failed\n";
                shutdown();
                return false;
            }
            _host->setPaceConfig(SyncPaceConfig{});
            _host->run();
        }

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
                if (syncSystem.requireIgConnect)
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
        if (_host)
        {
            _host->shutdown();
            _host.reset();
        }
        _role = {};
        resetEyeCaches();
    }

    void SynchronSystem::resetEyeCaches()
    {
        _hasPendingEye = false;
        _pendingEye = {};
        _cachedHostEye.reset();
        _lastApplied.reset();
        _lastSent.reset();
        _frameSample.reset();
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
            pose.position = vsg::dvec3(eye->x, eye->y, eye->z);
            pose.eulerYprDeg = vsg::dvec3(eye->yawDeg, eye->pitchDeg, eye->rollDeg);
            pose.frame = eye->isLla ? HostEyeCoordFrame::LLA : HostEyeCoordFrame::WORLD_LOCAL;
            queueHostEyePose(pose);
        }
    }

    void SynchronSystem::captureAuthorityEye(const vsg::LookAt& lookAt)
    {
        if (!_host)
            return;

        HostEyePose sample{};
        if (sceneIsEllipsoid())
        {
            if (!lookAtToLlaEye(lookAt, *_ellipsoidModel, sample))
                return;
        }
        else
        {
            if (!lookAtToWorldLocalEye(lookAt, sample))
                return;
        }

        // 防回声：把 LookAt 与 `_lastApplied` 重建比对（lla §4.4）；不减偏移。
        if (_lastApplied && lookAtMatchesApplied(lookAt, *_lastApplied, _ellipsoidModel.get()))
        {
            _frameSample.reset();
            return;
        }

        _frameSample = sample;
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
        const vsg::dquat qHost =
            vsg::dquat(vsg::radians(host.eulerYprDeg.z), vsg::dvec3(0.0, 1.0, 0.0)) *
            vsg::dquat(vsg::radians(host.eulerYprDeg.y), vsg::dvec3(1.0, 0.0, 0.0)) *
            vsg::dquat(vsg::radians(host.eulerYprDeg.x), vsg::dvec3(0.0, 0.0, 1.0));
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
        extractYprDegFromBasis(forward, up, out.eulerYprDeg);
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

    void SynchronSystem::postFrame(double simTimeMs)
    {
        if (!_host)
            return;

        const HostEyePose* sendEye = nullptr;
        HostEyePose eyeStorage{};
        const bool ellipsoid = sceneIsEllipsoid();

        if (_frameSample)
        {
            eyeStorage = *_frameSample;
            _frameSample.reset();
            const bool sampleLla = (eyeStorage.frame == HostEyeCoordFrame::LLA);
            if (sampleLla == ellipsoid)
                sendEye = &eyeStorage;
            // 否则丢弃不匹配的采样（若采样与场景匹配则不应发生）
        }
        else if (_lastSent)
        {
            const bool sentLla = (_lastSent->frame == HostEyeCoordFrame::LLA);
            if (sentLla != ellipsoid)
            {
                // lla §4.3：类型与场景不再匹配 —— 丢弃，不扇出。
                _lastSent.reset();
            }
            else
            {
                eyeStorage = *_lastSent;
                sendEye = &eyeStorage;
            }
        }

        HostSync::EyePose wire{};
        const HostSync::EyePose* wirePtr = nullptr;
        if (sendEye)
        {
            wire.x = sendEye->position.x;
            wire.y = sendEye->position.y;
            wire.z = sendEye->position.z;
            wire.yawDeg = sendEye->eulerYprDeg.x;
            wire.pitchDeg = sendEye->eulerYprDeg.y;
            wire.rollDeg = sendEye->eulerYprDeg.z;
            wire.isLla = (sendEye->frame == HostEyeCoordFrame::LLA);
            wirePtr = &wire;
            _lastSent = *sendEye;
        }

        _host->update(simTimeMs, wirePtr);
    }

    void SynchronSystem::setEllipsoidModel(vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid)
    {
        _ellipsoidModel = std::move(ellipsoid);
    }

    void SynchronSystem::setChannelId(int channelId)
    {
        _channelId = channelId;
    }

    std::optional<HostEyePose> SynchronSystem::takePendingCameraPose()
    {
        return std::exchange(_pendingApplied, std::nullopt);
    }

    HostSync& SynchronSystem::hostSync()
    {
        return *_host;
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

    void SynchronSystem::seedLastSentHostEye(const HostEyePose& pose)
    {
        _lastSent = pose;
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
