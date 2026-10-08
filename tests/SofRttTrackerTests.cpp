// 多通道同步验收测量设计.md §4.5 / §7：MEAS-sof-rtt 单 peer 配对（注入时钟）。

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/SofRttTracker.h>

#include <chrono>
#include <cstdint>

using aerovista::sync::SofRttTracker;

namespace
{
    std::chrono::steady_clock::time_point atMs(int ms)
    {
        return std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{ms};
    }
} // namespace

TEST_CASE("SofRttTracker records RTT when SOF matches Host Frame Number",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(7, atMs(0));
    tracker.onSofReceived(7, atMs(5));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(tracker.pendingCount() == 0);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{5});
}

TEST_CASE("SofRttTracker pairs by Host Frame Number not arrival order",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
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

TEST_CASE("SofRttTracker counts unmatched IGCtrl as loss at 100ms",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(3, atMs(0));
    tracker.expire(atMs(100));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(tracker.pendingCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

TEST_CASE("SofRttTracker treats SOF at 100ms as loss not RTT",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(4, atMs(0));
    tracker.onSofReceived(4, atMs(100));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

TEST_CASE("SofRttTracker matches SOF arriving before 100ms",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(5, atMs(0));
    tracker.onSofReceived(5, atMs(99));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{99});
}

TEST_CASE("SofRttTracker ignores SOF after match timeout",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(8, atMs(0));
    tracker.expire(atMs(100));
    tracker.onSofReceived(8, atMs(150));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

TEST_CASE("SofRttTracker ignores duplicate SOF for the same frame",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(9, atMs(0));
    tracker.onSofReceived(9, atMs(4));
    tracker.onSofReceived(9, atMs(8));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{4});
}

TEST_CASE("SofRttTracker ignores SOF for an unknown frame",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onSofReceived(99, atMs(3));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(tracker.pendingCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

TEST_CASE("SofRttTracker does not count in-flight IGCtrl as loss or RTT",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(10, atMs(0));
    tracker.expire(atMs(99));

    REQUIRE(tracker.pendingCount() == 1);
    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

TEST_CASE("SofRttTracker withholds avg until 10 matches",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
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

TEST_CASE("SofRttTracker avg uses the last 60 matches",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
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

TEST_CASE("SofRttTracker avg excludes losses",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    for (std::uint32_t frame = 1; frame <= 10; ++frame)
    {
        tracker.onIgCtrlSent(frame, atMs(0));
        tracker.onSofReceived(frame, atMs(4));
    }
    tracker.onIgCtrlSent(11, atMs(0));
    tracker.expire(atMs(100));

    REQUIRE(tracker.matchCount() == 10);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
    REQUIRE(tracker.lossRate().has_value());
    REQUIRE(*tracker.lossRate() == Catch::Approx(1.0 / 11.0));
}

TEST_CASE("SofRttTracker lossRate uses the last 60 completions",
          "[unit][sync][meas][MEAS-sof-rtt]")
{
    SofRttTracker tracker;
    tracker.onIgCtrlSent(1, atMs(0));
    tracker.expire(atMs(100));
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
