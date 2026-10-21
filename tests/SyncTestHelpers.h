#pragma once

// 跨测试 target（engine/Tests 与 thirdparty/sync/tests）共用的 E2E 辅助。
// 两边的 Common.h 各自 include 本头，避免 makeTest* / hostSendFrame / linkHostIg / collectFrames
// 在两个目录各写一份。TempConfigFile / EntitiesConfig 仍留在各自的 Common.h（engine 侧需要后者）。

#include <aerovista/config/EngineChannelConfig.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include <chrono>
#include <cstddef>
#include <functional>
#include <thread>
#include <vector>

using aerovista::sync::HostConfig;
using aerovista::sync::HostSync;
using aerovista::config::ViewDefinition;
using aerovista::sync::IgConfig;
using aerovista::sync::IgSync;

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

// =============================================================================
// E2E 动作辅助。
// =============================================================================

/// 业务侧扇出一帧 IGCtrl（outMsgWithIgCtrlUdp 自动前置 IGCtrl + 自计时时间戳）。
inline void hostSendFrame(HostSync& host)
{
    host.outMsgWithIgCtrlUdp();
    host.flushUdp();
}

/// 初始化 Host+IG 并完成握手。返回 true 表示流程走完；调用方若需要 IG 已 ready，
/// 自行 `REQUIRE(host.readyIgCount() == 1)`（不同测试对时序要求不同，这里不做强断言）。
inline bool linkHostIg(HostSync& host, IgSync& ig, int base)
{
    if (!host.initialize(makeTestHostConfig(base)))
        return false;
    const IgConfig igConfig = makeTestIgConfig(base + 1, base);
    if (!ig.initialize(igConfig.udpRecvPort) || !ig.connect(igConfig))
        return false;
    return true;
}

/// 轮询 take() 直到凑够 minCount 帧或 40×2ms 超时。用于等 I/O 线程入队的业务报文。
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
