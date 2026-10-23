// 多通道同步验收测量设计.md §4.5 / §4.8：RttTracker 配对与统计。
// 应用 RTT（§4.5，appRttMatchTimeout）与网络 RTT（§4.8，netRttMatchTimeout）共用 RttTracker，
// 同一码 MEAS-rtt 覆盖两张表。配对/窗逻辑与超时时限无关，共享用例只写一次；
// 边界用例按各自时限（50ms / 1ms）各测一次。真链路集成挂网络探测路径。

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/NetProbe.h>
#include <aerovista/sync/RttTracker.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <thread>

#include "Testutil.h"

using aerovista::sync::HostSync;
using aerovista::sync::IgSync;
using aerovista::sync::RttTracker;
using aerovista::sync::appRttMatchTimeout;
using aerovista::sync::netRttMatchTimeout;

namespace
{
    std::chrono::steady_clock::time_point atMs(int ms)
    {
        return std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{ms};
    }

    std::chrono::steady_clock::time_point atUs(int us)
    {
        return std::chrono::steady_clock::time_point{} + std::chrono::microseconds{us};
    }

    std::optional<std::chrono::microseconds> waitNetRtt(HostSync& host)
    {
        for (int i = 0; i < 40; ++i)
        {
            const auto peers = host.igSnapshot();
            if (!peers.empty())
            {
                if (auto sample = host.netRttLast(peers.front().channelId))
                    return sample;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }
        const auto peers = host.igSnapshot();
        if (peers.empty())
            return std::nullopt;
        return host.netRttLast(peers.front().channelId);
    }

    std::uint32_t waitSofReceived(HostSync& host)
    {
        // Host 主线程 drain 不等待；SOF 由 I/O 线程入队，这里轮询到这一条被解出来。
        for (int i = 0; i < 40; ++i)
        {
            host.drainIncoming();
            if (host.sofReceivedCount() > 0)
                return host.sofReceivedCount();
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }
        host.drainIncoming();
        return host.sofReceivedCount();
    }
} // namespace

// =============================================================================
// 回显键与发出键相同，核对 RTT 等于注入时钟的时间差（§4.5/§4.8）。
// 配对逻辑与超时时限无关，应用时限与网络时限共用；故两码同测。只验 lastRtt。
// =============================================================================
TEST_CASE("RttTracker records RTT when the echo key matches",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(7, atMs(0));
    tracker.onReceived(7, atMs(5));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{5});
}

// =============================================================================
// 后发的键先回显，核对 RTT 按键配对、不按到达顺序算给上一条（§4.5/§4.8）。
// 键 2 在 1ms 发出、6ms 回显 → 5ms；键 1 在 12ms 回显 → 12ms。
// =============================================================================
TEST_CASE("RttTracker pairs by key not arrival order",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(1, atMs(0));
    tracker.onSent(2, atMs(1));
    tracker.onReceived(2, atMs(6));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{5});

    tracker.onReceived(1, atMs(12));
    REQUIRE(tracker.matchCount() == 2);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{12});
}

// =============================================================================
// 已超时记丢失之后回显才到，核对不再补记 RTT（§4.5/§4.8）。
// 丢失只计一次；晚到不把时限或迟到间隔写进 lastRtt。
// =============================================================================
TEST_CASE("RttTracker ignores an echo after match timeout",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(8, atMs(0));
    tracker.expireInFlightAt(atMs(50));
    tracker.onReceived(8, atMs(150));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 同一键的第二次回显，核对不再改已记下的 RTT（§4.5/§4.8）。
// 第一次配对后表项已删除；第二次不增加 matchCount。
// =============================================================================
TEST_CASE("RttTracker ignores a duplicate echo for the same key",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(9, atMs(0));
    tracker.onReceived(9, atMs(4));
    tracker.onReceived(9, atMs(8));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{4});
}

// =============================================================================
// 没有对应发出记录的键，核对不产生匹配、丢失或 RTT（§4.5/§4.8）。
// 与晚到不同：表是空的，lossCount 保持 0。
// =============================================================================
TEST_CASE("RttTracker ignores an echo for an unknown key",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onReceived(99, atMs(3));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 未满 matchTimeout 的在途键，核对既不记丢失也不进 RTT（§4.5/§4.8）。
// expire 在 timeout-1：仍小于时限。完成窗空，lossRate 为空。
// =============================================================================
TEST_CASE("RttTracker does not count an in-flight key as loss or RTT",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(10, atMs(0));
    tracker.expireInFlightAt(atMs(49));

    REQUIRE(tracker.inFlightCount() == 1);
    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

// =============================================================================
// 窗内匹配数未满 avgMinSamples 时 avgRtt 为空，满 10 后才出（§4.5/§4.8）。
// 10 是出数门槛，不是窗长。这 10 次 RTT 都是 4ms。9 次时 lastRtt 已有值。
// =============================================================================
TEST_CASE("RttTracker withholds avg until 10 matches",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    for (std::uint32_t key = 1; key <= 9; ++key)
    {
        tracker.onSent(key, atMs(static_cast<int>(key) * 10));
        tracker.onReceived(key, atMs(static_cast<int>(key) * 10 + 4));
    }

    REQUIRE(tracker.matchCount() == 9);
    REQUIRE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.avgRtt().has_value());

    tracker.onSent(10, atMs(100));
    tracker.onReceived(10, atMs(104));
    REQUIRE(tracker.matchCount() == 10);
    REQUIRE(tracker.avgRtt().has_value());
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
}

// =============================================================================
// 超时占着完成窗的位置，但不参与 avgRtt 的平均（§4.5/§4.8）。
// 10 次 4ms 匹配再超时一次：avg 仍是 4ms；lossRate 按完成数算 = 1/11。
// =============================================================================
TEST_CASE("RttTracker avg excludes losses",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    for (std::uint32_t key = 1; key <= 10; ++key)
    {
        tracker.onSent(key, atMs(0));
        tracker.onReceived(key, atMs(4));
    }
    tracker.onSent(11, atMs(0));
    tracker.expireInFlightAt(atMs(50));

    REQUIRE(tracker.matchCount() == 10);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
    REQUIRE(tracker.lossRate().has_value());
    REQUIRE(*tracker.lossRate() == Catch::Approx(1.0 / 11.0));
}

// =============================================================================
// 完成数超过 60 后，核对 lossRate 只看最近 60 次完成，更早的超时滚出窗口（§4.5/§4.8）。
// 累计 lossCount 仍是 1；窗内 60 次都是 4ms 匹配，lossRate=0，avgRtt=4ms。
// =============================================================================
TEST_CASE("RttTracker lossRate uses the last 60 completions",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(1, atMs(0));
    tracker.expireInFlightAt(atMs(50));
    REQUIRE(tracker.lossCount() == 1);

    for (std::uint32_t key = 2; key <= 61; ++key)
    {
        tracker.onSent(key, atMs(0));
        tracker.onReceived(key, atMs(4));
    }

    REQUIRE(tracker.matchCount() == 60);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(tracker.lossRate().has_value());
    REQUIRE(*tracker.lossRate() == Catch::Approx(0.0));
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{4});
}

// =============================================================================
// 以下四条钉住 appRttMatchTimeout 边界（§4.5，码 MEAS-rtt）。网络 RTT 时限不同，另见下方 1ms 边界用例。
// =============================================================================

// =============================================================================
// 发出后满 50ms 仍无回显，核对该帧记丢失且不进 RTT（§4.5，码 MEAS-rtt）。
// 时限边界 now - tSend >= appRttMatchTimeout。完成数 1 < 10，lossRate 仍空。
// =============================================================================
TEST_CASE("App RTT counts an unmatched key as loss at 50ms",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(3, atMs(0));
    tracker.expireInFlightAt(atMs(50));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

// =============================================================================
// 回显时刻刚好等于 50ms，核对该帧记丢失且不进 RTT（§4.5，码 MEAS-rtt）。
// 走 onReceived，内部先按 tRecv 淘汰再查找；时限本身不得记成 RTT。
// =============================================================================
TEST_CASE("App RTT treats an echo at 50ms as loss not RTT",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(4, atMs(0));
    tracker.onReceived(4, atMs(50));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 回显比 50ms 早 1ms，核对仍按键配对且 RTT 为 49ms（§4.5，码 MEAS-rtt）。
// 与「刚好 50ms 记丢失」成对，钉住 >= 50ms：49ms 仍匹配。
// =============================================================================
TEST_CASE("App RTT matches an echo arriving before 50ms",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    tracker.onSent(5, atMs(0));
    tracker.onReceived(5, atMs(49));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(*tracker.lastRtt() == std::chrono::milliseconds{49});
}

// =============================================================================
// 完成数超过 60 后，核对 avgRtt 只平均最近 60 次完成里的匹配（§4.5/§4.8，码 MEAS-rtt）。
// 窗与时限无关，两码同测。样本须低于 appRttMatchTimeout：60 次 10ms 后第 61 次 40ms 挤掉最早的 10ms，
// avg = (59×10+40)/60 = 10.5ms。40ms 仍超过网络 1ms 时限，所以不在 1ms 追踪器上再写一条。
// =============================================================================
TEST_CASE("RttTracker avg uses the last 60 matches",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{appRttMatchTimeout};
    for (std::uint32_t key = 1; key <= 60; ++key)
    {
        tracker.onSent(key, atMs(0));
        tracker.onReceived(key, atMs(10));
    }
    REQUIRE(tracker.avgRtt().has_value());
    REQUIRE(*tracker.avgRtt() == std::chrono::milliseconds{10});

    tracker.onSent(61, atMs(0));
    tracker.onReceived(61, atMs(40));
    REQUIRE(tracker.matchCount() == 61);
    REQUIRE(*tracker.avgRtt() == std::chrono::microseconds{10500});
}

// =============================================================================
// 以下钉住 1ms 时限边界（§4.8，码 MEAS-rtt）。时钟用微秒，避免 1ms 以下被毫秒取整吃掉。
// =============================================================================

// =============================================================================
// 发出后满 1ms 仍无回显，核对该探测记丢失且不进 RTT（§4.8，码 MEAS-rtt）。
// 时限边界 now - tSend >= netRttMatchTimeout。完成数 1 < 10，lossRate 仍空。
// =============================================================================
TEST_CASE("Net RTT counts an unmatched probe as loss at 1ms",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{netRttMatchTimeout};
    tracker.onSent(3, atUs(0));
    tracker.expireInFlightAt(atUs(1000));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE(tracker.inFlightCount() == 0);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
    REQUIRE_FALSE(tracker.lossRate().has_value());
}

// =============================================================================
// 回显时刻刚好等于 1ms，核对该探测记丢失且不进 RTT（§4.8，码 MEAS-rtt）。
// 走 onReceived，内部先按 tRecv 淘汰再查找；时限本身不得记成 RTT。
// =============================================================================
TEST_CASE("Net RTT treats an echo at 1ms as loss not RTT",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{netRttMatchTimeout};
    tracker.onSent(4, atUs(0));
    tracker.onReceived(4, atUs(1000));

    REQUIRE(tracker.matchCount() == 0);
    REQUIRE(tracker.lossCount() == 1);
    REQUIRE_FALSE(tracker.lastRtt().has_value());
}

// =============================================================================
// 回显比 1ms 早 1us，核对仍按序号配对且 RTT 为 999us（§4.8，码 MEAS-rtt）。
// 与「刚好 1ms 记丢失」成对，钉住 >= 1ms：999us 仍匹配。
// =============================================================================
TEST_CASE("Net RTT matches an echo arriving before 1ms",
          "[unit][sync][meas][MEAS-rtt]")
{
    RttTracker tracker{netRttMatchTimeout};
    tracker.onSent(5, atUs(0));
    tracker.onReceived(5, atUs(999));

    REQUIRE(tracker.matchCount() == 1);
    REQUIRE(tracker.lossCount() == 0);
    REQUIRE(*tracker.lastRtt() == std::chrono::microseconds{999});
}

// =============================================================================
// 探测与 IGCtrl 一起发出，核对探测不使 IGCtrl / SOF 计数加一，IGCtrl 仍能被 IG 解包
// （§4.8，码 MEAS-rtt 分项 16）。一帧只应有 1 条 IGCtrl、1 条 SOF。网络 RTT 样本同时存在。
// Host 的 drainIncoming 不等 I/O 线程，所以要轮询到这一条 SOF 被解出来，仍然要求条数是 1。
// =============================================================================
SCENARIO("A network probe does not count as a CIGI business packet",
         "[integration][sync][meas][MEAS-rtt]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(linkHostIg(host, ig, 18680));
        REQUIRE(host.readyIgCount() == 1);

        WHEN("Host sends one data-plane frame and both sides unpack business packets")
        {
            hostSendFrame(host);
            const auto sample = waitNetRtt(host);
            ig.drainIncoming();
            const auto sofCount = waitSofReceived(host);

            THEN("IGCtrl and SOF stay one each, and the network sample is present")
            {
                REQUIRE(sample.has_value());
                REQUIRE(ig.igCtrlReceivedCount() == 1);
                REQUIRE(sofCount == 1);
            }
        }
    }
}

// =============================================================================
// 先收到探测回显，再解出 SOF，核对两套 RTT 各记各的（§4.8，码 MEAS-rtt 分项 17）。
// 解 SOF 之前网络样本已经有值、应用 RTT 仍为空，分项 15 的结果在这里一起看。
// 解完之后应用 RTT 有值，网络 RTT 保持解 SOF 之前的那个样本。
// 应用 RTT 要等 I/O 线程把 SOF 入队后主线程再解，所以 drain 会多试几次；样本值的要求不变。
// =============================================================================
SCENARIO("Network probe echoes and CIGI SOF update separate RTT samples",
         "[integration][sync][meas][MEAS-rtt]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(linkHostIg(host, ig, 18720));
        REQUIRE(host.readyIgCount() == 1);

        WHEN("the probe echo arrives and only afterwards the Host unpacks SOF")
        {
            hostSendFrame(host);
            const auto netBefore = waitNetRtt(host);
            const auto peers = host.igSnapshot();
            REQUIRE_FALSE(peers.empty());
            const auto appBefore = host.rttLast(peers.front().channelId);

            ig.drainIncoming(/*sendSof=*/true);
            std::optional<std::chrono::microseconds> netAfter;
            std::optional<std::chrono::microseconds> appAfter;
            for (int i = 0; i < 40; ++i)
            {
                host.drainIncoming();
                netAfter = host.netRttLast(peers.front().channelId);
                appAfter = host.rttLast(peers.front().channelId);
                if (appAfter)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds{2});
            }

            THEN("the probe sample is unchanged and the SOF sample is separate")
            {
                REQUIRE(netBefore.has_value());
                REQUIRE_FALSE(appBefore.has_value());
                REQUIRE(netAfter == netBefore);
                REQUIRE(appAfter.has_value());
                REQUIRE(*appAfter > std::chrono::microseconds{0});
            }
        }
    }
}
