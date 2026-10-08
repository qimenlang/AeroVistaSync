// viewhost 数值逻辑单元测试。
//
// 编码 doc/design/viewhost设计.md：
//   - §4.2 / §4.5 applyManualStep：键盘步进 → LLA / YPR 增量（机头局部参考系 +
//     绝对垂直；lat clamp / lon normalize / yaw normalize）。
//
// API 形状假设（实现方需满足）：
//   - applyManualStep(cigi_wire::EyePose&, dFwd, dRight, dUp, dyawDeg, dpitchDeg)。

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/CigiWire.h>

#include <cmath>

#include "ViewHostMath.h"

using aerovista::viewhost::applyManualStep;
namespace cigi_wire = aerovista::sync::cigi_wire;

namespace
{
    constexpr double kMetersPerDeg = 111320.0; // viewhost设计.md §4.2：1° lat ≈ 111320 m

    double degToRad(double deg)
    {
        return deg * (3.14159265358979323846 / 180.0);
    }

    bool nearlyEqual(double a, double b, double eps = 1e-6)
    {
        return std::abs(a - b) <= eps;
    }

    cigi_wire::EyePose makeLlaEye(double lat, double lon, double alt, double yaw = 0.0)
    {
        cigi_wire::EyePose eye;
        eye.x = lat;
        eye.y = lon;
        eye.z = alt;
        eye.yawDeg = yaw;
        return eye;
    }
} // namespace

// ===== applyManualStep（viewhost设计.md §4.2 / §4.5） =====

TEST_CASE("applyManualStep forward at yaw=0 increases latitude only", "[unit][viewhost][step][VH-step-forward]")
{
    auto eye = makeLlaEye(39.9, 116.4, 500.0, 0.0);

    applyManualStep(eye, /*dFwd=*/10.0, /*dRight=*/0.0, /*dUp=*/0.0, /*dyaw=*/0.0, /*dpitch=*/0.0);

    REQUIRE(nearlyEqual(eye.x, 39.9 + 10.0 / kMetersPerDeg));
    REQUIRE(nearlyEqual(eye.y, 116.4));
    REQUIRE(nearlyEqual(eye.z, 500.0));
}

TEST_CASE("applyManualStep strafe right at yaw=0 increases longitude scaled by cos(lat)", "[unit][viewhost][step][VH-step-strafe]")
{
    auto eye = makeLlaEye(60.0, 116.4, 500.0, 0.0); // cos(60°)=0.5 → 经度增量翻倍

    applyManualStep(eye, 0.0, /*dRight=*/10.0, 0.0, 0.0, 0.0);

    REQUIRE(nearlyEqual(eye.x, 60.0));
    REQUIRE(nearlyEqual(eye.y, 116.4 + 10.0 / (kMetersPerDeg * 0.5)));
}

TEST_CASE("applyManualStep forward at yaw=90 moves west", "[unit][viewhost][step][VH-step-yaw90]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0, 90.0); // §4.2：+yaw → 西

    applyManualStep(eye, /*dFwd=*/10.0, 0.0, 0.0, 0.0, 0.0);

    REQUIRE(nearlyEqual(eye.x, 30.0));
    REQUIRE(nearlyEqual(eye.y, 116.4 - 10.0 / (kMetersPerDeg * std::cos(degToRad(30.0)))));
}

TEST_CASE("applyManualStep forward at yaw=-90 moves east", "[unit][viewhost][step][VH-step-yaw-neg90]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0, -90.0); // §4.2：-yaw → 东

    applyManualStep(eye, /*dFwd=*/10.0, 0.0, 0.0, 0.0, 0.0);

    REQUIRE(nearlyEqual(eye.x, 30.0));
    REQUIRE(nearlyEqual(eye.y, 116.4 + 10.0 / (kMetersPerDeg * std::cos(degToRad(30.0)))));
}

TEST_CASE("applyManualStep up and down adjust altitude only", "[unit][viewhost][step][VH-step-alt]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0);

    applyManualStep(eye, 0.0, 0.0, /*dUp=*/20.0, 0.0, 0.0);
    REQUIRE(nearlyEqual(eye.z, 520.0));

    applyManualStep(eye, 0.0, 0.0, /*dUp=*/-20.0, 0.0, 0.0);
    REQUIRE(nearlyEqual(eye.z, 500.0));
}

TEST_CASE("applyManualStep accumulates yaw and pitch then clamps pitch", "[unit][viewhost][step][VH-step-clamp-pitch]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0, 0.0);

    applyManualStep(eye, 0.0, 0.0, 0.0, /*dyaw=*/10.0, /*dpitch=*/5.0);
    REQUIRE(nearlyEqual(eye.yawDeg, 10.0));
    REQUIRE(nearlyEqual(eye.pitchDeg, 5.0));

    applyManualStep(eye, 0.0, 0.0, 0.0, 0.0, /*dpitch=*/100.0); // 越界 → clamp
    REQUIRE(nearlyEqual(eye.pitchDeg, 89.9));
}

TEST_CASE("applyManualStep clamps latitude near the pole", "[unit][viewhost][step][VH-step-clamp-lat-n]")
{
    auto eye = makeLlaEye(89.9, 116.4, 500.0, 0.0); // 已在 clamp 上限

    applyManualStep(eye, /*dFwd=*/1000.0, 0.0, 0.0, 0.0, 0.0);

    REQUIRE(nearlyEqual(eye.x, 89.9)); // 被 clamp，而非 89.9 + 1000/111320
}

TEST_CASE("applyManualStep clamps latitude near the south pole", "[unit][viewhost][step][VH-step-clamp-lat-s]")
{
    auto eye = makeLlaEye(-89.9, 116.4, 500.0, 0.0); // 已在 clamp 下限

    applyManualStep(eye, /*dFwd=*/-1000.0, 0.0, 0.0, 0.0, 0.0);

    REQUIRE(nearlyEqual(eye.x, -89.9)); // 被 clamp，而非 -89.9 - 1000/111320
}

TEST_CASE("applyManualStep normalizes yaw to (-180, 180]", "[unit][viewhost][step][VH-norm-yaw]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0, 170.0);

    applyManualStep(eye, 0.0, 0.0, 0.0, /*dyaw=*/20.0, 0.0); // 170 + 20 = 190 → -170

    REQUIRE(nearlyEqual(eye.yawDeg, -170.0)); // 钉死 normalize 域，而非 angleNear（放过等价表示）
}

TEST_CASE("applyManualStep normalizes yaw across the +180 boundary", "[unit][viewhost][step][VH-norm-yaw-pos]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0, 180.0);

    applyManualStep(eye, 0.0, 0.0, 0.0, /*dyaw=*/20.0, 0.0); // 180 + 20 = 200 → -160

    REQUIRE(nearlyEqual(eye.yawDeg, -160.0));
}

TEST_CASE("applyManualStep normalizes negative yaw across the -180 boundary", "[unit][viewhost][step][VH-norm-yaw-neg]")
{
    auto eye = makeLlaEye(30.0, 116.4, 500.0, -170.0);

    applyManualStep(eye, 0.0, 0.0, 0.0, /*dyaw=*/-20.0, 0.0); // -170 - 20 = -190 → 170

    REQUIRE(nearlyEqual(eye.yawDeg, 170.0));
}

TEST_CASE("applyManualStep normalizes longitude across 180", "[unit][viewhost][step][VH-norm-lon]")
{
    auto eye = makeLlaEye(0.0, 179.9, 500.0, 0.0);

    // yaw=0 右移 dRight = 1°（cos(0)=1）：lon 180.9 → normalize 到 (-180, 180]
    applyManualStep(eye, 0.0, /*dRight=*/kMetersPerDeg, 0.0, 0.0, 0.0);

    REQUIRE(nearlyEqual(eye.y, -179.1));
}
