#include "ViewHostMath.h"

#include <cmath>

namespace aerovista::viewhost
{
    namespace
    {
        constexpr double kMetersPerDeg = 111320.0; // viewhost设计.md §4.2：1° lat ≈ 111320 m
        constexpr double kMaxLatDeg = 89.9;
        constexpr double kMaxPitchDeg = 89.9;
        constexpr double kPi = 3.14159265358979323846;

        double degToRad(double deg)
        {
            return deg * (kPi / 180.0);
        }

        /// 归一化到 (-180, 180]（lla位姿传输设计.md §5 / viewhost设计.md §4.2）。
        double normalizeDeg(double a)
        {
            double x = std::fmod(a, 360.0);
            if (x <= -180.0)
                x += 360.0;
            if (x > 180.0)
                x -= 360.0;
            return x;
        }

        double clampDeg(double v, double limit)
        {
            if (v > limit)
                return limit;
            if (v < -limit)
                return -limit;
            return v;
        }
    } // namespace

    void applyManualStep(aerovista::sync::HostSync::EyePose& eye,
                         double dFwd, double dRight, double dUp,
                         double dyawDeg, double dpitchDeg)
    {
        const double yawRad = degToRad(eye.yawDeg);
        const double sinYaw = std::sin(yawRad);
        const double cosYaw = std::cos(yawRad);

        // ENU 基（lla位姿传输设计.md §3.2）：X=East, Y=North。前=机头，右=机头右侧。
        const double forwardEast = -sinYaw;
        const double forwardNorth = cosYaw;
        const double rightEast = cosYaw;
        const double rightNorth = sinYaw;

        const double deltaNorth = forwardNorth * dFwd + rightNorth * dRight;
        const double deltaEast = forwardEast * dFwd + rightEast * dRight;

        const double lat = clampDeg(eye.x + deltaNorth / kMetersPerDeg, kMaxLatDeg);
        const double cosLat = std::cos(degToRad(lat));

        eye.x = lat;
        eye.y = normalizeDeg(eye.y + deltaEast / (kMetersPerDeg * cosLat));
        eye.z += dUp;
        eye.yawDeg = normalizeDeg(eye.yawDeg + dyawDeg);
        eye.pitchDeg = clampDeg(eye.pitchDeg + dpitchDeg, kMaxPitchDeg);
    }
} // namespace aerovista::viewhost
