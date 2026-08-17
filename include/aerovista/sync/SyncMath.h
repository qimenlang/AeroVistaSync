#pragma once

#include <cstddef>

/// aerovistaSync 门面层的公开边界类型：纯 POD + 注入接口，零 vsg 依赖。
/// 作用：让消费方（含完全无 vsg 的 viewhost）在编译/链接期都不接触 vsg 头文件。
/// 内部实现可复用 vsg header-only 数学（见 SynchronSystem.cpp），但不在公开头暴露。
namespace aerovista::sync
{
    /// 纯 POD 三维向量（公开边界类型，替代公开签名里的 vsg::dvec3）。
    struct DVec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    /// POD 版 LookAt（替代公开签名里的 vsg::LookAt）。
    struct CameraLookAt
    {
        DVec3 eye{};
        DVec3 center{};
        DVec3 up{};
    };

    /// 大地测量学转换，由宿主实现并注入（engine 用 vsg::EllipsoidModel 实现）。
    /// sync 库只声明、只调用，不持有所有权（宿主保证生命周期覆盖 sync 会话）。
    /// 注入非空 = 椭球模式，空 = 本地模式（与 SynchronSystem::sceneIsEllipsoid 语义一致）。
    struct EllipsoidTransform
    {
        virtual ~EllipsoidTransform() = default;

        /// ECEF（地心直角坐标，米）→ LLA（纬度°、经度°、海拔 米）。
        virtual DVec3 ecefToLla(const DVec3& ecef) const = 0;

        /// LLA（纬度°、经度°、海拔 米）→ ECEF（米）。
        virtual DVec3 llaToEcef(const DVec3& lla) const = 0;

        /// 给定 LLA 处的 ENU 基三轴（east/north/up），等价
        /// vsg::computeLocalToWorldTransform 的正交列。用于 ENU↔ECEF 方向换算。
        virtual void localToWorldBasis(const DVec3& lla, DVec3& east, DVec3& north, DVec3& up) const = 0;
    };
} // namespace aerovista::sync
