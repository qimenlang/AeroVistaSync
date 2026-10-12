// 多通道同步验收测量设计.md §4.5 / §7：MEAS-rtt 单 peer 配对（注入时钟）。

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/AppRttTracker.h>

#include <chrono>
#include <cstdint>

using aerovista::sync::AppRttTracker;

namespace
{
    std::chrono::steady_clock::time_point atMs(int ms)
    {
        return std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{ms};
    }
} // namespace

// =============================================================================
// 回显帧号与发出的 Host Frame Number 相同，核对应用 RTT 是否等于注入时钟的时间差
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 时钟由 atMs 注入，不走墙钟。只验 lastRtt；avgRtt 要满 10 个匹配样本，本用例不验。
// =============================================================================
TEST_CASE("AppRttTracker records RTT when the echo matches Host Frame Number",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(7, atMs(0));
    tracker.onSofReceived(7, atMs(5));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{5});
}

// =============================================================================
// 后发的帧先回显，核对 RTT 按 Host Frame Number 配对，不按到达顺序把回显算给上一条
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 帧 2 在 1ms 发出、6ms 回显，配对结果是 5ms；若按到达顺序算给帧 1，会得到 6ms。
// 帧 1 随后在 12ms 回显，RTT 仍相对它自己的 0ms 发出时刻。
// =============================================================================
TEST_CASE("AppRttTracker pairs by Host Frame Number not arrival order",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(1, atMs(0));
    tracker.onIgCtrlSent(2, atMs(1));
    tracker.onSofReceived(2, atMs(6));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{5});

    tracker.onSofReceived(1, atMs(12));
    REQUIRE(tracker.matchCount() == 2);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{12});
}

// =============================================================================
// 发出后满 matchTimeout 仍无回显，核对该帧记为丢失且不进 RTT
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 时限边界是 now - tSend >= 100ms。丢失不写入 lastRtt。
// 完成数只有 1，不足 avgMinSamples（10），故 lossRate 仍为空。
// =============================================================================
TEST_CASE("AppRttTracker counts unmatched IGCtrl as loss at 100ms",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(3, atMs(0));
    tracker.expireInFlightAt(atMs(100));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

// =============================================================================
// 回显时刻刚好等于 matchTimeout，核对该帧记丢失且不进 RTT
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 与上一条不同：上一条只调用 expireInFlightAt；这里走 onSofReceived，内部先按 tRecv 淘汰再查找。
// 时限本身不得记成 RTT。
// =============================================================================
TEST_CASE("AppRttTracker treats an echo at 100ms as loss not RTT",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(4, atMs(0));
    tracker.onSofReceived(4, atMs(100));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 回显比 matchTimeout 早 1ms，核对仍按帧号配对且 RTT 为 99ms
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 与「刚好 100ms 记丢失」成对，钉住 now - tSend >= 100ms：99ms 仍匹配。
// 5ms 那条只说明窗内能配对，不卡边界。
// =============================================================================
TEST_CASE("AppRttTracker matches an echo arriving before 100ms",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(5, atMs(0));
    tracker.onSofReceived(5, atMs(99));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{99});
}

// =============================================================================
// 已超时记丢失之后回显才到，核对不再补记 RTT
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 丢失只计一次。晚到不把时限或迟到间隔写进 lastRtt。
// =============================================================================
TEST_CASE("AppRttTracker ignores an echo after match timeout",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(8, atMs(0));
    tracker.expireInFlightAt(atMs(100));
    tracker.onSofReceived(8, atMs(150));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 同一帧号的第二次回显，核对不再改已记下的 RTT
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 第一次 4ms 配对后表项已删除。不要求第二次把 matchCount 再加一。
// =============================================================================
TEST_CASE("AppRttTracker ignores a duplicate echo for the same frame",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(9, atMs(0));
    tracker.onSofReceived(9, atMs(4));
    tracker.onSofReceived(9, atMs(8));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{4});
}

// =============================================================================
// 没有对应发出记录的帧号，核对不产生匹配、丢失或 RTT
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 与晚到不同：晚到之前已经记过一次丢失；这里表是空的，lossCount 保持 0。
// =============================================================================
TEST_CASE("AppRttTracker ignores an echo for an unknown frame",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onSofReceived(99, atMs(3));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 未满 matchTimeout 的在途帧，核对既不记丢失也不进 RTT
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// expireInFlightAt(99ms) 仍小于 100ms。这里没有回显，完成窗是空的，故 lossRate 为空。
// =============================================================================
TEST_CASE("AppRttTracker does not count in-flight IGCtrl as loss or RTT",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(10, atMs(0));
    tracker.expireInFlightAt(atMs(99));

    REQUIRE(tracker.inFlightCount() == 1);
    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

// =============================================================================
// 窗内匹配数未满 avgMinSamples 时 avgRtt 为空，满 10 个后才给出这些匹配的平均
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 滚动窗是最近 60 次完成；10 是出数门槛，不是窗长。这 10 次 RTT 都是 4ms。
// 9 次时 lastRtt 已经有值。
// =============================================================================
TEST_CASE("AppRttTracker withholds avg until 10 matches",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    for (std::uint32_t frame = 1; frame <= 9; ++frame)
    {
        tracker.onIgCtrlSent(frame, atMs(static_cast<int>(frame) * 10));
        tracker.onSofReceived(frame, atMs(static_cast<int>(frame) * 10 + 4));
    }

    REQUIRE(tracker.matchCount() == 9);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.avgRtt().has_value());

    tracker.onIgCtrlSent(10, atMs(100));
    tracker.onSofReceived(10, atMs(104));
    REQUIRE(tracker.matchCount() == 10);
    REQUIRE(tracker.avgRtt().has_value());
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
}

// =============================================================================
// 完成数超过 avgWindow 后，核对 avgRtt 只平均最近 60 次完成里的匹配样本
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 本用例没有丢失，所以这 60 次全是匹配。第 61 次 70ms 挤掉最早的 10ms，
// 平均变为 (59×10+70)/60 = 11ms。
// =============================================================================
TEST_CASE("AppRttTracker avg uses the last 60 matches",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    for (std::uint32_t frame = 1; frame <= 60; ++frame)
    {
        tracker.onIgCtrlSent(frame, atMs(0));
        tracker.onSofReceived(frame, atMs(10));
    }
    REQUIRE(tracker.avgRtt().has_value());
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{10});

    tracker.onIgCtrlSent(61, atMs(0));
    tracker.onSofReceived(61, atMs(70));
    REQUIRE(tracker.matchCount() == 61);
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{11});
}

// =============================================================================
// 超时占着完成窗的位置，但不参与 avgRtt 的平均
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 先凑满 10 次 4ms 匹配再超时一次：平均仍是 4ms。lossRate 按完成数算，是 1/11。
// =============================================================================
TEST_CASE("AppRttTracker avg excludes losses",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    for (std::uint32_t frame = 1; frame <= 10; ++frame)
    {
        tracker.onIgCtrlSent(frame, atMs(0));
        tracker.onSofReceived(frame, atMs(4));
    }
    tracker.onIgCtrlSent(11, atMs(0));
    tracker.expireInFlightAt(atMs(100));

    REQUIRE(tracker.matchCount() == 10);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
    REQUIRE(tracker.lossRate().has_value());
    REQUIRE(*tracker.lossRate() == Catch::Approx(1.0 / 11.0));
}

// =============================================================================
// 完成数超过 60 后，核对 lossRate 只看最近 60 次完成，更早的超时滚出窗口
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 累计 lossCount 仍是 1。窗口里剩下的 60 次都是 4ms 匹配，所以 lossRate 为 0，avgRtt 为 4ms。
// =============================================================================
TEST_CASE("AppRttTracker lossRate uses the last 60 completions",
          "[unit][sync][meas][MEAS-rtt]")
{
    AppRttTracker tracker;
    tracker.onIgCtrlSent(1, atMs(0));
    tracker.expireInFlightAt(atMs(100));
    REQUIRE(tracker.lossCount() == 1);

    for (std::uint32_t frame = 2; frame <= 61; ++frame)
    {
        tracker.onIgCtrlSent(frame, atMs(0));
        tracker.onSofReceived(frame, atMs(4));
    }

    REQUIRE(tracker.matchCount() == 60);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(tracker.lossRate().has_value());
    REQUIRE(*tracker.lossRate() == Catch::Approx(0.0));
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
}
