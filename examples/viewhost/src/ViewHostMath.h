#pragma once

#include <aerovista/sync/CigiWire.h>

namespace aerovista::viewhost
{
    /// 键盘步进 → 眼点增量（viewhost设计.md §4.2 / §4.5）。
    ///
    /// 前置条件：`eye.frame == EyeFrame::LLA`（调用方保证；步进在 LLA 语义下进行）。
    /// - dFwd/dRight/dUp：机头局部前/右/上位移（米）。前右在机头水平面内随 yaw 旋转，上为绝对垂直。
    /// - dyawDeg/dpitchDeg：姿态增量（度）。
    /// 结果：lat clamp 到 [-89.9, 89.9]、lon/yaw normalize 到 (-180, 180]、
    ///       pitch clamp 到 [-89.9, 89.9]、alt 直接累加。
    void applyManualStep(aerovista::sync::cigi_wire::EyePose& eye,
                         double dFwd, double dRight, double dUp,
                         double dyawDeg, double dpitchDeg);
} // namespace aerovista::viewhost
