#pragma once

// sync 测试辅助：临时配置、端口、握手和线格式动作。

#include <catch2/catch_test_macros.hpp>

#include <aerovista/config/EngineChannelConfig.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

using aerovista::config::ViewDefinition;
using aerovista::sync::HostConfig;
using aerovista::sync::HostSync;
using aerovista::sync::IgConfig;
using aerovista::sync::IgSync;

class TempConfigFile
{
public:
    explicit TempConfigFile(const std::string& jsonBody)
    {
        _path = (std::filesystem::temp_directory_path() /
                 ("ave_engine_cfg_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json"))
                    .string();
        std::ofstream out(_path, std::ios::binary);
        out << jsonBody;
    }

    ~TempConfigFile()
    {
        std::error_code ec;
        std::filesystem::remove(_path, ec);
    }

    const std::string& path() const { return _path; }

private:
    std::string _path;
};

// =============================================================================
// Host/IG 网络角色配置辅助。各测试文件传不同 base 隔离端口，避免并行冲突。
// =============================================================================

inline HostConfig makeTestHostConfig(int base)
{
    return HostConfig{base, base + 100};
}

inline IgConfig makeTestIgConfig(int udpRecvPort, int base)
{
    return IgConfig{udpRecvPort, "127.0.0.1", base + 100, base};
}

inline ViewDefinition makeTestViewDefinition(int viewID, bool requireConnectedIg = true)
{
    ViewDefinition viewDefinition;
    viewDefinition.viewID = viewID;
    viewDefinition.requireConnectedIg = requireConnectedIg;
    return viewDefinition;
}

// 默认端口见 doc/design/多通道同步/多通道同步模块设计.md
inline HostConfig makeHostLocal()
{
    return HostConfig{8000, 8100};
}

inline IgConfig makeIgLocal(int udpRecvPort = 8001)
{
    return IgConfig{udpRecvPort, "127.0.0.1", 8100, 8000};
}

// =============================================================================
// E2E 动作辅助。
// =============================================================================

// IGCtrl 由 outMsgWithIgCtrlUdp() 自动前置。
inline void hostSendFrame(HostSync& host)
{
    host.outMsgWithIgCtrlUdp();
    host.flushUdp();
}

inline void hostSendEyePose(HostSync& host, const aerovista::sync::cigi_wire::EyePose& eye)
{
    auto& omsg = host.outMsgWithIgCtrlUdp();
    aerovista::sync::cigi_wire::appendEye(omsg, &eye);
    host.flushUdp();
}

// 初始化 Host+IG 并完成握手。返回 true 表示流程走完；调用方若需要 IG 已 ready，自行断言。
inline bool linkHostIg(HostSync& host, IgSync& ig, int base)
{
    if (!host.initialize(makeTestHostConfig(base)))
        return false;
    const IgConfig igConfig = makeTestIgConfig(base + 1, base);
    if (!ig.initialize(igConfig.udpRecvPort) || !ig.connect(igConfig))
        return false;
    return true;
}

// 默认端口上一对已握手的 Host 与 IG。连接成败另有用例，这里只当前置。
inline void setupHostIgPair(HostSync& host, IgSync& ig)
{
    REQUIRE(host.initialize(makeHostLocal()));
    REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
    REQUIRE(ig.connect(makeIgLocal()));
}

inline bool startHostTwoIgs(HostSync& host, IgSync& master, IgSync& side, int base)
{
    if (!host.initialize(makeTestHostConfig(base)))
        return false;
    const IgConfig masterCfg = makeTestIgConfig(base + 1, base);
    const IgConfig sideCfg = makeTestIgConfig(base + 3, base);
    if (!master.initialize(masterCfg.udpRecvPort, 0))
        return false;
    if (!side.initialize(sideCfg.udpRecvPort, 1))
        return false;
    if (!master.connect(masterCfg))
        return false;
    if (!side.connect(sideCfg))
        return false;
    return host.readyIgCount() == 2;
}

// 条件成立即返回。短轮询只盖住 Host 就绪表晚于 IG connect 返回的缝，不空等固定超时。
inline void connectUntil(const std::function<bool()>& linked)
{
    for (int i = 0; i < 10 && !linked(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

inline bool framesIncreasing(const std::vector<std::uint32_t>& frames)
{
    for (std::size_t i = 1; i < frames.size(); ++i)
    {
        if (frames[i] <= frames[i - 1])
            return false;
    }
    return true;
}

// UDP 可丢：actual 落在 [expected-slack, expected]。
inline bool approxAtMost(std::uint32_t actual, int expected, int slack)
{
    const auto exp = static_cast<std::uint32_t>(expected);
    const auto minOk = exp > static_cast<std::uint32_t>(slack) ? exp - static_cast<std::uint32_t>(slack) : 0u;
    return actual >= minOk && actual <= exp;
}

// 轮询 take() 直到凑够 minCount 帧或 40×2ms 超时。用于等 I/O 线程入队的业务报文。
inline std::vector<std::vector<unsigned char>> collectFrames(
    const std::function<std::vector<std::vector<unsigned char>>()>& take, std::size_t minCount)
{
    std::vector<std::vector<unsigned char>> frames;
    for (int i = 0; i < 40 && frames.size() < minCount; ++i)
    {
        auto more = take();
        frames.insert(frames.end(), more.begin(), more.end());
        if (frames.size() < minCount)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return frames;
}

struct OwnshipEyeCapture
{
    bool got = false;
    std::uint16_t entityId = 0;
    std::uint16_t parentId = 0;
    CigiBaseEntityPositionCtrl::AttachStateGrp attachState = CigiBaseEntityPositionCtrl::Detach;
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double yawDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
};

inline void captureOwnship(IgSync& ig, OwnshipEyeCapture& cap)
{
    ig.addCallback<CigiEntityPositionCtrlV4>([&](const CigiEntityPositionCtrlV4& pose) {
        if (pose.GetEntityID() != 0)
            return;
        cap.got = true;
        cap.entityId = pose.GetEntityID();
        cap.parentId = pose.GetParentID();
        cap.attachState = pose.GetAttachState();
        cap.lat = pose.GetLat();
        cap.lon = pose.GetLon();
        cap.alt = pose.GetAlt();
        cap.yawDeg = pose.GetYaw();
        cap.pitchDeg = pose.GetPitch();
        cap.rollDeg = pose.GetRoll();
    });
}

inline void pumpIgCtrlFrames(HostSync& host, IgSync& ig, int frames = 5)
{
    for (int i = 0; i < frames; ++i)
    {
        hostSendFrame(host);
        ig.drainIncoming();
        ig.update();
    }
}

inline void pumpOwnshipEyeFrames(HostSync& host, IgSync& ig, const aerovista::sync::cigi_wire::EyePose& eye, int frames = 5)
{
    for (int i = 0; i < frames; ++i)
    {
        hostSendEyePose(host, eye);
        ig.drainIncoming();
        ig.update();
    }
}
