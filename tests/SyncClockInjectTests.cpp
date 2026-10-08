#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/IgSync.h>

#include <cstdint>

using aerovista::sync::IgSync;

namespace
{
    using HostTimeStamp = IgSync::HostTimeStamp;
} // namespace

SCENARIO("linked IG reports simulation time as Host base plus local elapsed",
         "[acceptance][bdd][sync][clock][CLK-consume]")
{
    GIVEN("an IgSync that has received one Host timestamp")
    {
        IgSync ig;
        const std::uint32_t raw = 1000; // 10000 us
        const std::uint64_t receivedAtUs = 20000;
        ig.queueHostTimeStamp(HostTimeStamp{100, raw, receivedAtUs});

        WHEN("the consumer asks for simulation time later in the same frame")
        {
            const std::uint64_t nowUs = 26000; // 本地流逝 6000us

            THEN("simTimeUs equals Host base plus local elapsed")
            {
                // 首包 lastSimTimeUs = raw*10 = 10000us，加上 (nowUs - receivedAtUs)
                REQUIRE(ig.simTimeUsAt(nowUs) == raw * 10 + (nowUs - receivedAtUs));
            }
        }
    }
}

SCENARIO("IG takes the last of multiple same-frame time stamps",
         "[acceptance][bdd][sync][clock][CLK-latest][CLK-same-frame]")
{
    GIVEN("an IgSync that receives two packets for the same frame")
    {
        IgSync ig;
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000}); // 10000us @20000us
        ig.queueHostTimeStamp(HostTimeStamp{100, 1500, 24000}); // 15000us @24000us

        WHEN("the consumer asks for simulation time")
        {
            const std::uint64_t nowUs = 26000;

            THEN("the last of the same-frame packets is the base")
            {
                REQUIRE(ig.simTimeUsAt(nowUs) == 1500 * 10 + (nowUs - 24000));
            }
        }
    }
}

SCENARIO("IG rejects an older-frame time stamp entirely",
         "[acceptance][bdd][sync][clock][CLK-old-frame][CLK-reorder]")
{
    GIVEN("an IgSync that received frame 100 then an older frame 99")
    {
        IgSync ig;
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000});
        ig.queueHostTimeStamp(HostTimeStamp{99, 999, 21000});

        WHEN("the consumer asks for simulation time")
        {
            const std::uint64_t nowUs = 22000;

            THEN("the older frame does not move the base")
            {
                REQUIRE(ig.simTimeUsAt(nowUs) == 1000 * 10 + (nowUs - 20000));
            }
        }
    }
}

SCENARIO("IG accepts a skipped frame number as a fresh base",
         "[acceptance][bdd][sync][clock][CLK-skip-frame]")
{
    GIVEN("an IgSync that receives frame 100 then a skipped frame 102")
    {
        IgSync ig;
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000});
        ig.queueHostTimeStamp(HostTimeStamp{102, 2000, 40000}); // 101 帧跳过

        WHEN("the consumer asks for simulation time")
        {
            const std::uint64_t nowUs = 41000;

            THEN("frame 102 is accepted and becomes the base")
            {
                REQUIRE(ig.simTimeUsAt(nowUs) == 2000 * 10 + (nowUs - 40000));
            }
        }
    }
}

SCENARIO("IG extrapolates simulation time with local elapsed while no frame arrives",
         "[acceptance][bdd][sync][clock][CLK-extrapolate]")
{
    GIVEN("an IgSync with a large extrapolate timeout that received one frame")
    {
        IgSync ig;
        ig.setExtrapolateTimeoutUs(1000000); // 1s
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000});

        WHEN("time passes with no new frame")
        {
            THEN("simTimeUs keeps increasing with local elapsed")
            {
                REQUIRE(ig.simTimeUsAt(30000) == 1000 * 10 + 10000);
                REQUIRE(ig.simTimeUsAt(35000) == 1000 * 10 + 15000);
            }
        }
    }
}

SCENARIO("IG freezes beyond extrapolate timeout and returns a constant simulation time",
         "[acceptance][bdd][sync][clock][CLK-freeze]")
{
    GIVEN("an IgSync with a 50ms timeout that received one frame")
    {
        IgSync ig;
        ig.setExtrapolateTimeoutUs(50000); // 50ms
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000});

        WHEN("more than the timeout elapses without a frame")
        {
            ig.updateFreeze(80000); // nowUs - lastReceivedAtUs = 60000 > 50000 → 冻结

            THEN("the IG enters frozen state and simTimeUs stays constant")
            {
                REQUIRE(ig.frozen());
                REQUIRE(ig.simTimeUsAt(80000) == 1000 * 10);
                REQUIRE(ig.simTimeUsAt(90000) == 1000 * 10);
            }
        }
    }
}

SCENARIO("IG does not freeze within the extrapolate timeout",
         "[acceptance][bdd][sync][clock][CLK-freeze]")
{
    GIVEN("an IgSync with a 50ms timeout that received one frame")
    {
        IgSync ig;
        ig.setExtrapolateTimeoutUs(50000);
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000});

        WHEN("less than the timeout elapses")
        {
            ig.updateFreeze(60000); // nowUs - lastReceivedAtUs = 40000 < 50000

            THEN("the IG stays not-frozen and keeps compensating elapsed time")
            {
                REQUIRE_FALSE(ig.frozen());
                REQUIRE(ig.simTimeUsAt(60000) == 1000 * 10 + (60000 - 20000));
            }
        }
    }
}

SCENARIO("IG jumps directly to a new base after freeze recovery",
         "[acceptance][bdd][sync][clock][CLK-unfreeze]")
{
    GIVEN("an IgSync frozen after its timeout")
    {
        IgSync ig;
        ig.setExtrapolateTimeoutUs(50000);
        ig.queueHostTimeStamp(HostTimeStamp{100, 1000, 20000});
        ig.updateFreeze(80000); // 冻结状态
        REQUIRE(ig.frozen());

        WHEN("a new frame arrives after the freeze")
        {
            ig.queueHostTimeStamp(HostTimeStamp{101, 9000, 90000});

            THEN("freeze clears and simTimeUs jumps to the new base")
            {
                REQUIRE_FALSE(ig.frozen());
                REQUIRE(ig.simTimeUsAt(91000) == 9000 * 10 + (91000 - 90000));
            }
        }
    }
}

SCENARIO("IG phase unwrap crosses a single 2^32 wrap without a jump",
         "[acceptance][bdd][sync][clock][wrap][CLK-wrap]")
{
    GIVEN("an IgSync whose raw time stamps are about to wrap")
    {
        IgSync ig;
        ig.queueHostTimeStamp(HostTimeStamp{100, 0xfffffff0u, 10000});

        WHEN("the next stamp wraps past the uint32 limit")
        {
            ig.queueHostTimeStamp(HostTimeStamp{101, 0x10u, 11000}); // +32 tick

            THEN("extended time increases by exactly 32 ticks, no jump")
            {
                // 首包 base = 0xfffffff0*10 us，第二包 base = (0xfffffff0+32)*10 us
                REQUIRE(ig.simTimeUsAt(11000) == (static_cast<std::uint64_t>(0xfffffff0u) + 32) * 10);
            }
        }
    }
}

SCENARIO("IG phase unwrap stays monotonic across multiple wraps",
         "[acceptance][bdd][sync][clock][wrap][CLK-wrap-multi]")
{
    GIVEN("an IgSync receiving stamps that wrap several times")
    {
        IgSync ig;
        ig.queueHostTimeStamp(HostTimeStamp{100, 0xfffffff0u, 10000});

        WHEN("stamps keep increasing across the uint32 limit")
        {
            std::uint64_t prev = 0;
            bool first = true;
            for (std::uint32_t i = 0; i < 5; ++i)
            {
                // 每次接近回绕值：越过回绕值 - i，小值 + i
                ig.queueHostTimeStamp(HostTimeStamp{101 + i * 2, static_cast<std::uint32_t>(0xfffffff0u - i), 10000 + i * 100});
                ig.queueHostTimeStamp(HostTimeStamp{102 + i * 2, static_cast<std::uint32_t>(0x10u + i), 10000 + i * 100 + 50});
                const std::uint64_t cur = ig.simTimeUsAt(10000 + i * 100 + 60);
                if (!first)
                    REQUIRE(cur > prev);
                prev = cur;
                first = false;
            }
        }
    }
}

TEST_CASE("session reset after Host restart starts from the small raw without inheriting the old base",
          "[unit][sync][clock][session][CLK-session-restart]")
{
    // 时钟同步方案.md §3 边界：Host 重启后 → TCP 重连 → 新会话基准，首包 raw 为小值。
    // resetHostSession() 使相位展开状态回到新基准，不继承旧会话大值（否则 simTimeUs 超 2^32）。
    IgSync ig;
    ig.queueHostTimeStamp(HostTimeStamp{1, 0xf0000000u, 10000}); // 旧会话大值
    ig.queueHostTimeStamp(HostTimeStamp{2, 0xf0000010u, 20000});

    ig.resetHostSession();                                  // 会话重置（重连 TCP 时调用）
    ig.queueHostTimeStamp(HostTimeStamp{1, 0x100u, 30000}); // 新会话首包小值

    // 新基准 = 0x100*10 us，不继承旧会话 0xf0000010 大值。
    REQUIRE(ig.simTimeUsAt(30000) == 0x100u * 10);
}
