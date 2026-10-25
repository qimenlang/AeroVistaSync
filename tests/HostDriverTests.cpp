#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/CigiIncludes.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/HostDriver.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiCollDetSegDefV4.h"
#include "CigiCollDetSegRespV4.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiIGCtrlV4.h"
#include "CigiIGMsgV4.h"
#include "CigiSOFV4.h"
#include "CigiSymbolTextDefV4.h"
#include "CigiWeatherCtrlV4.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <optional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Testutil.h"

using aerovista::sync::HostConfig;
using aerovista::sync::HostDriver;
using aerovista::sync::HostSync;
using aerovista::sync::IgConfig;
using aerovista::sync::IgSync;
namespace cigi_wire = aerovista::sync::cigi_wire;

// 平台中继：HostDriver 转发，以及用两条直连手工转发来核对线载荷。
// 直连 Host 的 channelId / master TCP 过滤在 HostIGLinkTests.cpp。

namespace
{
    HostConfig makeRelayViewhostConfig(int viewhostBase, int platformBase, int expectedIgCount)
    {
        HostConfig cfg = makeTestHostConfig(viewhostBase);
        cfg.relay.enable = true;
        cfg.relay.expectedIgCount = expectedIgCount;
        cfg.igConfig = makeTestIgConfig(platformBase + 2, platformBase);
        return cfg;
    }

    void tickRelay(HostDriver& viewhost, int ticks = 20)
    {
        for (int i = 0; i < ticks; ++i)
        {
            viewhost.pollRelay();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void sendCollDetSegDef(HostSync& host)
    {
        auto& tcp = host.outMsgWithIgCtrlTcp();
        CigiCollDetSegDefV4 def;
        def.SetEntityID(7);
        def.SetSegmentEn(true);
        tcp << def;
        host.flushTcp();
    }

    void sendCollDetSegRespUdp(IgSync& ig, std::uint32_t material)
    {
        auto& udp = ig.outMsgWithSofUdp();
        CigiCollDetSegRespV4 resp;
        resp.SetEntityID(7);
        resp.SetMaterial(material);
        udp << resp;
        ig.flushUdp();
    }

    bool hasMaterial(const std::vector<std::uint32_t>& got, std::uint32_t want)
    {
        for (auto material : got)
        {
            if (material == want)
                return true;
        }
        return false;
    }

    bool materialsInOrder(const std::vector<std::uint32_t>& got, std::initializer_list<std::uint32_t> want)
    {
        std::size_t i = 0;
        for (auto material : want)
        {
            while (i < got.size() && got[i] != material)
                ++i;
            if (i == got.size())
                return false;
            ++i;
        }
        return true;
    }

    bool waitVirtualIgHandshakeDone(HostDriver& viewhost, HostSync& platform)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
        while ((!viewhost.virtualIgHandshakeDone() || platform.readyIgCount() != 1) &&
               std::chrono::steady_clock::now() < deadline)
        {
            viewhost.pollRelay();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return viewhost.virtualIgHandshakeDone() && platform.readyIgCount() == 1;
    }

    bool startHostDriverRelay(HostSync& platform, HostDriver& viewhost, IgSync& realIg, int platformBase,
                              int viewhostBase)
    {
        if (!platform.initialize(makeTestHostConfig(platformBase)))
            return false;
        if (!viewhost.initialize(makeRelayViewhostConfig(viewhostBase, platformBase, 1)))
            return false;
        const IgConfig realCfg = makeTestIgConfig(viewhostBase + 1, viewhostBase);
        if (!realIg.initialize(realCfg.udpRecvPort, 0))
            return false;
        if (!realIg.connect(realCfg))
            return false;
        return waitVirtualIgHandshakeDone(viewhost, platform);
    }

    bool startHostDriverRelayTwoIgs(HostSync& platform, HostDriver& viewhost, IgSync& master, IgSync& side,
                                    int platformBase, int viewhostBase)
    {
        if (!platform.initialize(makeTestHostConfig(platformBase)))
            return false;
        if (!viewhost.initialize(makeRelayViewhostConfig(viewhostBase, platformBase, 2)))
            return false;
        const IgConfig masterCfg = makeTestIgConfig(viewhostBase + 1, viewhostBase);
        const IgConfig sideCfg = makeTestIgConfig(viewhostBase + 3, viewhostBase);
        if (!master.initialize(masterCfg.udpRecvPort, 0))
            return false;
        if (!side.initialize(sideCfg.udpRecvPort, 1))
            return false;
        if (!master.connect(masterCfg))
            return false;
        if (!side.connect(sideCfg))
            return false;
        return waitVirtualIgHandshakeDone(viewhost, platform);
    }

    void drainIgUntil(IgSync& ig, const std::function<bool()>& done, bool sendSof = false)
    {
        for (int i = 0; i < 40 && !done(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            ig.drainIncoming(sendSof);
        }
    }

    void drainHostUntil(HostSync& host, const std::function<bool()>& done)
    {
        for (int i = 0; i < 40 && !done(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            host.drainIncoming();
        }
    }

    void drainDriverUntil(HostDriver& driver, const std::function<bool()>& done)
    {
        for (int i = 0; i < 40 && !done(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            driver.pollIncoming();
        }
    }

    // UDP 可丢：重发直到观察到；全丢由调用方 SKIP。
    template<typename Send>
    bool retryUdpUntil(Send send, const std::function<bool()>& done, int attempts = 10)
    {
        for (int i = 0; i < attempts && !done(); ++i)
            send();
        return done();
    }

    std::vector<unsigned char> concatFrames(const std::vector<unsigned char>& a,
                                            const std::vector<unsigned char>& b)
    {
        std::vector<unsigned char> out = a;
        out.insert(out.end(), b.begin(), b.end());
        return out;
    }

    cigi_wire::EyePose makeEye(double lat, double lon, double alt, double yaw, double pitch)
    {
        cigi_wire::EyePose eye{};
        eye.x = lat;
        eye.y = lon;
        eye.z = alt;
        eye.yawDeg = yaw;
        eye.pitchDeg = pitch;
        return eye;
    }

    void requireOwnship(const OwnshipEyeCapture& cap, const cigi_wire::EyePose& eye)
    {
        REQUIRE(cap.lat == Catch::Approx(eye.x));
        REQUIRE(cap.lon == Catch::Approx(eye.y));
        REQUIRE(cap.alt == Catch::Approx(eye.z));
        REQUIRE(cap.yawDeg == Catch::Approx(eye.yawDeg));
        REQUIRE(cap.pitchDeg == Catch::Approx(eye.pitchDeg));
    }

    OwnshipEyeCapture ownshipFrom(const CigiEntityPositionCtrlV4& pose)
    {
        OwnshipEyeCapture cap;
        cap.got = true;
        cap.entityId = pose.GetEntityID();
        cap.lat = pose.GetLat();
        cap.lon = pose.GetLon();
        cap.alt = pose.GetAlt();
        cap.yawDeg = pose.GetYaw();
        cap.pitchDeg = pose.GetPitch();
        cap.rollDeg = pose.GetRoll();
        return cap;
    }

    void sendTcpSymbol(HostSync& host, const char* text)
    {
        auto& tcp = host.outMsgWithIgCtrlTcp();
        CigiSymbolTextDefV4 cmd(text);
        tcp << cmd;
        host.flushTcp();
    }

    void sendIgTcpReport(IgSync& ig, std::uint16_t msgId, const char* text)
    {
        auto& tcp = ig.outMsgWithSofTcp();
        CigiIGMsgV4 report;
        report.SetMsgID(msgId);
        report.SetMsg(text);
        tcp << report;
        ig.flushTcp();
    }

    bool relayPlatformOwnship(HostSync& platform, HostDriver& viewhost, IgSync& ig, const cigi_wire::EyePose& eye)
    {
        // 回调留在 IgSync 上；捕获放堆上，函数返回后后续眼点包不会写到已销毁的栈对象。
        auto cap = std::make_shared<OwnshipEyeCapture>();
        ig.addCallback<CigiEntityPositionCtrlV4>([cap](const CigiEntityPositionCtrlV4& pose) {
            if (pose.GetEntityID() != 0)
                return;
            *cap = ownshipFrom(pose);
        });
        return retryUdpUntil(
            [&] {
                hostSendEyePose(platform, eye);
                viewhost.pollRelay();
                drainIgUntil(ig, [&] { return cap->got; });
            },
            [&] { return cap->got; });
    }
} // namespace

// =============================================================================
// 两台 IG 都发 TCP 上报时，平台只收到 master 的载荷（平台同步设计.md §12，码 PLT-hello-ch）。
// 两条分开 flush。不按包号决定转发。
// =============================================================================
SCENARIO("relay forwards only the master IG TCP report to the platform",
         "[acceptance][platform][PLT-hello-ch]")
{
    GIVEN("a platform Host, a viewhost relay, and two ready real IGs on channel 0 and 1")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync master;
        IgSync side;
        REQUIRE(startHostDriverRelayTwoIgs(platform, viewhost, master, side, 46000, 46200));

        std::vector<std::string> platformMsgs;
        platform.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            platformMsgs.emplace_back(const_cast<CigiIGMsgV4&>(msg).GetMsg());
        });

        WHEN("both IGs send distinct TCP reports and the relay ticks")
        {
            // 两条独立 flush；握手后仅 master TCP 入队，不认 PacketID。
            {
                auto& tcp = master.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3101);
                report.SetMsg("master");
                tcp << report;
                master.flushTcp();
            }
            {
                auto& tcp = side.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3102);
                report.SetMsg("side");
                tcp << report;
                side.flushTcp();
            }
            tickRelay(viewhost);
            drainHostUntil(platform, [&] { return !platformMsgs.empty(); });
            drainHostUntil(platform, [&] { return platformMsgs.size() >= 2; });

            THEN("the platform receives only the master payload")
            {
                REQUIRE(platformMsgs.size() == 1);
                REQUIRE(platformMsgs.front() == "master");
            }
        }
    }
}

// =============================================================================
// 只有侧通道发 TCP 上报时，平台什么都收不到（平台同步设计.md §12，码 PLT-hello-ch）。
// master 在线但不发。不按包号过滤。
// =============================================================================
SCENARIO("relay does not forward a side-channel IG TCP report to the platform",
         "[acceptance][platform][PLT-hello-ch]")
{
    GIVEN("a platform Host, a viewhost relay, and two ready real IGs on channel 0 and 1")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync master;
        IgSync side;
        REQUIRE(startHostDriverRelayTwoIgs(platform, viewhost, master, side, 48000, 48200));

        std::vector<std::string> platformMsgs;
        platform.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            platformMsgs.emplace_back(const_cast<CigiIGMsgV4&>(msg).GetMsg());
        });

        WHEN("only the side IG sends a TCP report and the relay ticks")
        {
            // 反向用例：master 在线但不发；侧通道 TCP 不入队，不认 PacketID。
            {
                auto& tcp = side.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3103);
                report.SetMsg("side-only");
                tcp << report;
                side.flushTcp();
            }
            tickRelay(viewhost);
            drainHostUntil(platform, [&] { return !platformMsgs.empty(); });

            THEN("the platform receives no IG TCP report")
            {
                REQUIRE(platformMsgs.empty());
            }
        }
    }
}

// =============================================================================
// 实 IG 还没到齐时，虚 IG 不向平台握手（平台同步设计.md §12，码 PLT-ig-first）。
// 期望两台，只到一台。平台 ready 为 0。
// =============================================================================
SCENARIO("virtual IG does not join the platform before real IGs have gathered",
         "[acceptance][platform][PLT-ig-first]")
{
    GIVEN("a platform Host and a viewhost relay expecting two real IGs")
    {
        constexpr int kPlatform = 41000;
        constexpr int kViewhost = 41200;
        HostSync platform;
        REQUIRE(platform.initialize(makeTestHostConfig(kPlatform)));

        HostDriver viewhost;
        REQUIRE(viewhost.initialize(makeRelayViewhostConfig(kViewhost, kPlatform, 2)));

        AND_GIVEN("only one real IG is ready")
        {
            IgSync realIg;
            const IgConfig realCfg = makeTestIgConfig(kViewhost + 1, kViewhost);
            REQUIRE(realIg.initialize(realCfg.udpRecvPort, 0));
            REQUIRE(realIg.connect(realCfg));
            REQUIRE(viewhost.readyIgCount() == 1);

            WHEN("the relay keeps ticking")
            {
                tickRelay(viewhost);

                THEN("the virtual IG has not connected and the platform has no ready IG")
                {
                    REQUIRE_FALSE(viewhost.virtualIgHandshakeDone());
                    REQUIRE(platform.readyIgCount() == 0);
                }
            }
        }
    }
}

// =============================================================================
// 实 IG 到齐后，虚 IG 以通道 0 加入平台，并且不算 viewhost 的对端（平台同步设计.md §12，码 PLT-not-peer）。
// 平台 ready 为 1。viewhost 快照里仍是两台实 IG。
// =============================================================================
SCENARIO("gathered real IGs let the virtual IG join the platform without becoming a viewhost peer",
         "[acceptance][platform][PLT-not-peer]")
{
    GIVEN("a platform Host and a viewhost relay expecting two real IGs")
    {
        constexpr int kPlatform = 41400;
        constexpr int kViewhost = 41600;
        HostSync platform;
        REQUIRE(platform.initialize(makeTestHostConfig(kPlatform)));

        HostDriver viewhost;
        REQUIRE(viewhost.initialize(makeRelayViewhostConfig(kViewhost, kPlatform, 2)));

        AND_GIVEN("both real IGs are ready")
        {
            IgSync real0;
            IgSync real1;
            const IgConfig cfg0 = makeTestIgConfig(kViewhost + 1, kViewhost);
            const IgConfig cfg1 = makeTestIgConfig(kViewhost + 3, kViewhost);
            REQUIRE(real0.initialize(cfg0.udpRecvPort, 0));
            REQUIRE(real1.initialize(cfg1.udpRecvPort, 1));
            REQUIRE(real0.connect(cfg0));
            REQUIRE(real1.connect(cfg1));
            REQUIRE(viewhost.readyIgCount() == 2);

            WHEN("the relay ticks after gather")
            {
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
                while ((!viewhost.virtualIgHandshakeDone() || platform.readyIgCount() != 1) &&
                       std::chrono::steady_clock::now() < deadline)
                {
                    viewhost.pollRelay();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                THEN("the platform has one ready IG on channel 0, and viewhost snapshot lists only the real IGs")
                {
                    REQUIRE(viewhost.virtualIgHandshakeDone());
                    REQUIRE(platform.readyIgCount() == 1);
                    const auto platformPeers = platform.igSnapshot();
                    REQUIRE(platformPeers.size() == 1);
                    REQUIRE(platformPeers.front().channelId == 0);

                    const auto realPeers = viewhost.igSnapshot();
                    REQUIRE(realPeers.size() == 2);
                    REQUIRE((realPeers[0].channelId == 0 || realPeers[1].channelId == 0));
                    REQUIRE((realPeers[0].channelId == 1 || realPeers[1].channelId == 1));
                    REQUIRE(realPeers[0].channelId != realPeers[1].channelId);
                }
            }
        }
    }
}

// =============================================================================
// 转到实 IG 的 ownship 是平台眼点，不是 viewhost 键盘眼点（平台同步设计.md §12，码 PLT-filter-ownship）。
// EntityID 为 0。UDP 全部丢失则跳过。
// =============================================================================
SCENARIO("relayed platform ownship is the IG eye, not the viewhost keyboard",
         "[acceptance][platform][PLT-filter-ownship]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready real IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 41800, 42000));

        OwnshipEyeCapture cap;
        captureOwnship(realIg, cap);

        cigi_wire::EyePose platformEye{};
        platformEye.x = 31.2;
        platformEye.y = 121.5;
        platformEye.z = 80.0;
        platformEye.yawDeg = 45.0;
        platformEye.pitchDeg = 5.0;

        cigi_wire::EyePose keyboardEye{};
        keyboardEye.x = 1.0;
        keyboardEye.y = 2.0;
        keyboardEye.z = 3.0;
        keyboardEye.yawDeg = 10.0;
        keyboardEye.pitchDeg = 20.0;

        WHEN("the platform sends ownship and the relay ticks while viewhost still has a keyboard eye")
        {
            const bool observed = retryUdpUntil(
                [&] {
                    hostSendEyePose(platform, platformEye);
                    viewhost.update(&keyboardEye);
                    viewhost.pollRelay();
                    drainIgUntil(realIg, [&] { return cap.got; });
                },
                [&] { return cap.got; });
            if (!observed)
                SKIP("UDP datagram dropped");

            THEN("the IG ownship is the platform packet, not the keyboard pose")
            {
                REQUIRE(cap.entityId == 0);
                REQUIRE(cap.lat == Catch::Approx(platformEye.x));
                REQUIRE(cap.lon == Catch::Approx(platformEye.y));
                REQUIRE(cap.alt == Catch::Approx(platformEye.z));
                REQUIRE(cap.yawDeg == Catch::Approx(platformEye.yawDeg));
                REQUIRE(cap.pitchDeg == Catch::Approx(platformEye.pitchDeg));
            }
        }
    }
}

// =============================================================================
// 平台的 UDP IGCtrl 原样转发，viewhost 不再另发一帧数据面（平台同步设计.md §12，码 PLT-filter-igctrl）。
// viewhost 的 igCtrlSentCount 为 0。实 IG 收到的条数不超过平台发出数。丢包则跳过。
// =============================================================================
SCENARIO("HostDriver relay forwards platform UDP IGCtrl without adding another data-plane frame",
         "[acceptance][platform][PLT-filter-igctrl]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready real IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 42200, 42400));

        std::vector<std::uint32_t> igFrames;
        std::vector<bool> igTimeStampValid;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            igFrames.push_back(ctrl.GetFrameCntr());
            igTimeStampValid.push_back(ctrl.GetTimeStampValid());
        });

        cigi_wire::EyePose keyboardEye{};
        keyboardEye.x = 1.0;
        keyboardEye.y = 2.0;
        keyboardEye.z = 3.0;

        WHEN("the platform sends UDP IGCtrl and the relay ticks with a keyboard eye still available")
        {
            const bool observed = retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    viewhost.update(&keyboardEye);
                    viewhost.pollRelay();
                    drainIgUntil(realIg, [&] { return !igFrames.empty(); });
                },
                [&] { return !igFrames.empty(); });
            if (!observed)
                SKIP("UDP datagram dropped");

            THEN("the IG sees only platform data-plane IGCtrl, not an extra viewhost frame")
            {
                REQUIRE(viewhost.igCtrlSentCount() == 0);
                REQUIRE(realIg.igCtrlReceivedCount() <= platform.igCtrlSentCount());
                REQUIRE(realIg.igCtrlReceivedCount() == igFrames.size());
                for (bool valid : igTimeStampValid)
                    REQUIRE(valid);
            }
        }
    }
}

// =============================================================================
// 平台 TCP 命令的原载荷到达实 IG（平台同步设计.md §12，码 PLT-tcp-pass）。
// TimeStampValid 为 false。
// =============================================================================
SCENARIO("HostDriver relay forwards a platform TCP command with the original payload",
         "[acceptance][platform][PLT-tcp-pass]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready real IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 42600, 42800));

        bool igTimeStampValid = true;
        std::string igText;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            igTimeStampValid = ctrl.GetTimeStampValid();
        });
        realIg.addCallback<CigiSymbolTextDefV4>([&](const CigiSymbolTextDefV4& txt) {
            igText = const_cast<CigiSymbolTextDefV4&>(txt).GetText();
        });

        WHEN("the platform sends a TCP command and the relay ticks")
        {
            {
                auto& tcp = platform.outMsgWithIgCtrlTcp();
                CigiSymbolTextDefV4 cmd("pass");
                tcp << cmd;
                platform.flushTcp();
            }
            tickRelay(viewhost);
            drainIgUntil(realIg, [&] { return !igText.empty(); });

            THEN("the real IG sees the platform payload and TimeStampValid")
            {
                REQUIRE(igText == "pass");
                REQUIRE(igTimeStampValid == false);
            }
        }
    }
}

// =============================================================================
// 实 IG 的 TCP 上报到达平台（平台同步设计.md §12，码 PLT-tcp-return）。
// 报文号是 0x3001，正文是发出的那串。
// =============================================================================
SCENARIO("HostDriver relay forwards an IG TCP report to the platform",
         "[acceptance][platform][PLT-tcp-return]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready real IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 43000, 43200));

        std::uint16_t platformMsgId = 0;
        std::string platformMsg;
        platform.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            platformMsgId = msg.GetMsgID();
            platformMsg = const_cast<CigiIGMsgV4&>(msg).GetMsg();
        });

        WHEN("the real IG reports on TCP and the relay ticks")
        {
            {
                auto& tcp = realIg.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3001);
                report.SetMsg("up");
                tcp << report;
                realIg.flushTcp();
            }
            tickRelay(viewhost);
            drainHostUntil(platform, [&] { return !platformMsg.empty(); });

            THEN("the platform sees the IG message")
            {
                REQUIRE(platformMsgId == 0x3001);
                REQUIRE(platformMsg == "up");
            }
        }
    }
}

// =============================================================================
// 碰撞段命中经中继按序到达平台（平台同步设计.md §12，码 PLT-return-collision）。
// 材质顺序是 1、2、3。命中时的 SOF 帧号不是 0，来自虚 IG。实 IG 自己的 IGCtrl 帧号仍是 0。
// =============================================================================
SCENARIO("platform receives repeating collision-segment notifications through the relay",
         "[acceptance][platform][PLT-return-collision]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready master IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync master;
        REQUIRE(startHostDriverRelay(platform, viewhost, master, 50400, 50600));

        std::optional<CigiCollDetSegDefV4> igDef;
        std::vector<std::uint32_t> platformMaterials;
        std::optional<std::uint32_t> lastSofFrame;
        std::optional<std::uint32_t> sofAtHit;
        master.addCallback<CigiCollDetSegDefV4>([&](const CigiCollDetSegDefV4& def) { igDef = def; });
        platform.addCallback<CigiSOFV4>([&](const CigiSOFV4& sof) { lastSofFrame = sof.GetFrameCntr(); });
        platform.addCallback<CigiCollDetSegRespV4>([&](const CigiCollDetSegRespV4& resp) {
            sofAtHit = lastSofFrame;
            platformMaterials.push_back(resp.GetMaterial());
        });

        WHEN("the platform defines a segment and the master reports three UDP hits")
        {
            sendCollDetSegDef(platform);
            tickRelay(viewhost);
            drainIgUntil(master, [&] { return igDef.has_value(); });
            REQUIRE(igDef.has_value());

            // 虚 IG 先 packSof 非 0 帧号；不 drain master，真实 SOF 仍为 0。
            REQUIRE(retryUdpUntil(
                [&] {
                    hostSendFrame(platform);
                    tickRelay(viewhost, 5);
                },
                [&] {
                    platform.drainIncoming();
                    return lastSofFrame.has_value() && *lastSofFrame != 0;
                }));

            REQUIRE(retryUdpUntil(
                [&] {
                    sendCollDetSegRespUdp(master, 1);
                    sendCollDetSegRespUdp(master, 2);
                    sendCollDetSegRespUdp(master, 3);
                    tickRelay(viewhost, 5);
                },
                [&] {
                    platform.drainIncoming();
                    return platformMaterials.size() >= 3;
                }));

            THEN("the platform receives the hits in order on SOF'")
            {
                REQUIRE(materialsInOrder(platformMaterials, {1u, 2u, 3u}));
                REQUIRE(sofAtHit.has_value());
                REQUIRE(*sofAtHit != 0);
                REQUIRE(master.lastIgCtrlFrameCntr() == 0);
            }
        }
    }
}

// =============================================================================
// 侧通道的碰撞通知不到平台（平台同步设计.md §12，码 PLT-return-collision）。
// 平台只有 master 的材质。
// =============================================================================
SCENARIO("relay keeps a side-channel collision notification off the platform",
         "[acceptance][platform][PLT-return-collision]")
{
    GIVEN("a platform Host, a viewhost relay, and two ready real IGs on channel 0 and 1")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync master;
        IgSync side;
        REQUIRE(startHostDriverRelayTwoIgs(platform, viewhost, master, side, 50800, 51000));

        std::vector<std::uint32_t> platformMaterials;
        platform.addCallback<CigiCollDetSegRespV4>(
            [&](const CigiCollDetSegRespV4& resp) { platformMaterials.push_back(resp.GetMaterial()); });

        WHEN("the master and the side IG both send UDP collision notifications")
        {
            REQUIRE(retryUdpUntil(
                [&] {
                    sendCollDetSegRespUdp(master, 11);
                    sendCollDetSegRespUdp(side, 22);
                    tickRelay(viewhost, 5);
                },
                [&] {
                    platform.drainIncoming();
                    return hasMaterial(platformMaterials, 11);
                }));

            THEN("the platform receives only the master notification")
            {
                REQUIRE(hasMaterial(platformMaterials, 11));
                REQUIRE_FALSE(hasMaterial(platformMaterials, 22));
            }
        }
    }
}

// =============================================================================
// 两台实 IG 的 UDP SOF 留在 viewhost，平台收不到（平台同步设计.md §12，码 PLT-relay-sof）。
// viewhost 的 SOF 计数增加，平台的计数不变。丢包则跳过。
// =============================================================================
SCENARIO("relay keeps real IG UDP SOF from both channels on viewhost",
         "[acceptance][platform][PLT-relay-sof]")
{
    GIVEN("a platform Host, a viewhost relay, and two ready real IGs on channel 0 and 1")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync master;
        IgSync side;
        REQUIRE(startHostDriverRelayTwoIgs(platform, viewhost, master, side, 47600, 47800));

        const auto platformSofBefore = platform.sofReceivedCount();
        const auto viewhostSofBefore = viewhost.sofReceivedCount();

        WHEN("both IGs send UDP SOF and the relay ticks")
        {
            const bool observed = retryUdpUntil(
                [&] {
                    master.outMsgWithSofUdp();
                    master.flushUdp();
                    side.outMsgWithSofUdp();
                    side.flushUdp();
                    tickRelay(viewhost, 5);
                    drainDriverUntil(viewhost, [&] { return viewhost.sofReceivedCount() > viewhostSofBefore; });
                    drainHostUntil(platform, [&] { return platform.sofReceivedCount() > platformSofBefore; });
                },
                [&] { return viewhost.sofReceivedCount() > viewhostSofBefore; });
            if (!observed)
                SKIP("UDP datagram dropped");

            THEN("the platform does not receive those SOF")
            {
                REQUIRE(viewhost.sofReceivedCount() > viewhostSofBefore);
                REQUIRE(platform.sofReceivedCount() == platformSofBefore);
            }
        }
    }
}

// =============================================================================
// UDP 转发之后，平台看到的 SOF 只来自虚 IG 的 packSof（平台同步设计.md §12，码 PLT-relay-sof）。
// 帧号不是 0xffffffff。丢包则跳过。
// =============================================================================
SCENARIO("HostDriver relay packSof after UDP forward is the only SOF the platform sees",
         "[acceptance][platform][PLT-relay-sof]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready real IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 43400, 43600));

        std::uint32_t platformSof = 0xffffffffu;
        platform.addCallback<CigiSOFV4>([&](const CigiSOFV4& sof) { platformSof = sof.GetFrameCntr(); });

        WHEN("the platform sends one UDP IGCtrl and the relay ticks")
        {
            const bool observed = retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    tickRelay(viewhost, 5);
                    drainHostUntil(platform, [&] { return platform.sofReceivedCount() > 0; });
                },
                [&] { return platform.sofReceivedCount() > 0; });
            if (!observed)
                SKIP("UDP datagram dropped");
            drainIgUntil(realIg, [&] { return realIg.igCtrlReceivedCount() > 0; }, true);
            drainDriverUntil(viewhost, [&] { return viewhost.sofReceivedCount() > 0; });
            const auto sofAtStop = platform.sofReceivedCount();
            drainHostUntil(platform, [&] { return false; });

            THEN("the platform receives SOF from the virtual IG, not a second SOF from the real IG")
            {
                REQUIRE(platform.sofReceivedCount() == sofAtStop);
                REQUIRE(platformSof != 0xffffffffu);
            }
        }
    }
}

// =============================================================================
// 没有就绪实 IG 时，虚 IG 仍 packSof，平台能收到（平台同步设计.md §12，码 PLT-relay-sof）。
// 虚 IG 握手还在。丢包则跳过。
// =============================================================================
SCENARIO("HostDriver relay packSof even when no real IG is ready",
         "[acceptance][platform][PLT-relay-sof]")
{
    GIVEN("a linked virtual IG whose real IG has dropped")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 43800, 44000));
        realIg.shutdown();
        for (int i = 0; i < 40 && viewhost.readyIgCount() != 0; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        REQUIRE(viewhost.readyIgCount() == 0);
        REQUIRE(viewhost.virtualIgHandshakeDone());

        std::uint32_t platformSof = 0xffffffffu;
        platform.addCallback<CigiSOFV4>([&](const CigiSOFV4& sof) { platformSof = sof.GetFrameCntr(); });

        WHEN("the platform sends UDP IGCtrl and the relay ticks")
        {
            const bool observed = retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    tickRelay(viewhost, 5);
                    drainHostUntil(platform, [&] { return platform.sofReceivedCount() > 0; });
                },
                [&] { return platform.sofReceivedCount() > 0; });
            if (!observed)
                SKIP("UDP datagram dropped");

            THEN("the platform still receives a SOF")
            {
                REQUIRE(platform.sofReceivedCount() >= 1);
                REQUIRE(platformSof != 0xffffffffu);
            }
        }
    }
}

// =============================================================================
// viewhost 自己的 TCP 命令不打断平台 UDP 帧号（平台同步设计.md §12，码 PLT-splice-cmd）。
// 到达的 UDP 帧号保持递增，TCP 正文单独是那条命令。丢包则跳过。
// =============================================================================
SCENARIO("HostDriver viewhost TCP command does not break platform UDP frame numbers",
         "[acceptance][platform][PLT-splice-cmd]")
{
    GIVEN("a platform Host, a viewhost relay, and one ready real IG")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 44200, 44400));

        std::vector<std::uint32_t> igUdpFrames;
        std::string igText;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            if (ctrl.GetTimeStampValid())
                igUdpFrames.push_back(ctrl.GetFrameCntr());
        });
        realIg.addCallback<CigiSymbolTextDefV4>([&](const CigiSymbolTextDefV4& txt) {
            igText = const_cast<CigiSymbolTextDefV4&>(txt).GetText();
        });

        WHEN("the platform sends UDP IGCtrl, viewhost sends its own TCP command, then another UDP IGCtrl")
        {
            const bool observedUdp = retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    viewhost.pollRelay();
                    drainIgUntil(realIg, [&] { return !igUdpFrames.empty(); });
                },
                [&] { return !igUdpFrames.empty(); });
            REQUIRE(viewhost.sendSymbolText("wx"));
            drainIgUntil(realIg, [&] { return !igText.empty(); });
            if (!observedUdp)
                SKIP("UDP datagram dropped");

            THEN("arrived UDP FrameCntr stay increasing and the TCP command is separate")
            {
                REQUIRE(framesIncreasing(igUdpFrames));
                REQUIRE(igText == "wx");
            }
        }
    }
}

// =============================================================================
// 转发后的平台 TCP 命令保持原载荷（平台同步设计.md §12，码 PLT-tcp-pass）。
// TimeStampValid 仍是 false。与走 HostDriver 的那条共用码，这里直接转分帧后的字节。
// =============================================================================
SCENARIO("relayed platform TCP command keeps the original payload",
         "[acceptance][platform][PLT-tcp-pass]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 36000));
        REQUIRE(linkHostIg(viewhost, realIg, 36200));

        bool igTimeStampValid = true;
        std::string igText;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            igTimeStampValid = ctrl.GetTimeStampValid();
        });
        realIg.addCallback<CigiSymbolTextDefV4>([&](const CigiSymbolTextDefV4& txt) {
            igText = const_cast<CigiSymbolTextDefV4&>(txt).GetText();
        });

        WHEN("the platform sends a TCP command and the relay forwards the framed bytes")
        {
            {
                auto& tcp = platform.outMsgWithIgCtrlTcp();
                CigiSymbolTextDefV4 cmd("pass");
                tcp << cmd;
                platform.flushTcp();
            }

            const auto frames = collectFrames([&] { return virtualIg.takeIncomingTcp(); }, 1);
            REQUIRE_FALSE(frames.empty());
            viewhost.sendTcpMessage(frames.back());

            for (int i = 0; i < 40 && igText.empty(); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                realIg.drainIncoming(false);
            }

            THEN("the real IG sees the platform payload and TimeStampValid, not a rewritten header")
            {
                REQUIRE(igText == "pass");
                REQUIRE(igTimeStampValid == false);
            }
        }
    }
}

// =============================================================================
// 一次 TCP 写里粘住的两条 Host 消息被拆开转发（平台同步设计.md §12，码 PLT-frame-msg）。
// 实 IG 收到帧号 10 然后 11。
// =============================================================================
SCENARIO("relay splits two Host TCP messages glued in one sendAll",
         "[acceptance][platform][PLT-frame-msg]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 36400));
        REQUIRE(linkHostIg(viewhost, realIg, 36600));

        std::vector<std::uint32_t> igFrames;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            igFrames.push_back(ctrl.GetFrameCntr());
        });

        WHEN("platform sends two complete IGCtrl messages in one TCP write and the relay forwards each framed vector")
        {
            std::vector<unsigned char> first;
            std::vector<unsigned char> second;
            REQUIRE(cigi_wire::packIgCtrl(10, first));
            REQUIRE(cigi_wire::packIgCtrl(11, second));
            platform.sendTcpMessage(concatFrames(first, second));

            const auto frames = collectFrames([&] { return virtualIg.takeIncomingTcp(); }, 2);
            REQUIRE(frames.size() >= 2);
            viewhost.sendTcpMessage(frames[0]);
            viewhost.sendTcpMessage(frames[1]);

            for (int i = 0; i < 40 && igFrames.size() < 2; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                realIg.drainIncoming(false);
            }

            THEN("the real IG receives two separate TCP messages")
            {
                REQUIRE(igFrames.size() == 2);
                REQUIRE(igFrames[0] == 10);
                REQUIRE(igFrames[1] == 11);
            }
        }
    }
}

// =============================================================================
// 一次 TCP 写里粘住的两条 IG 消息被拆开转发（平台同步设计.md §12，码 PLT-frame-msg）。
// 平台收到 SOF 帧号 20 然后 21。
// =============================================================================
SCENARIO("relay splits two IG TCP messages glued in one sendAll",
         "[acceptance][platform][PLT-frame-msg]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 36800));
        REQUIRE(linkHostIg(viewhost, realIg, 37000));

        std::vector<std::uint32_t> platformSof;
        platform.addCallback<CigiSOFV4>([&](const CigiSOFV4& sof) {
            platformSof.push_back(sof.GetFrameCntr());
        });

        WHEN("the real IG sends two complete SOF messages in one TCP write and the relay forwards each framed vector")
        {
            std::vector<unsigned char> first;
            std::vector<unsigned char> second;
            REQUIRE(cigi_wire::packSof(20, first));
            REQUIRE(cigi_wire::packSof(21, second));
            realIg.sendTcpMessage(concatFrames(first, second));

            const auto frames = collectFrames([&] { return viewhost.takeIncomingTcp(); }, 2);
            REQUIRE(frames.size() >= 2);
            virtualIg.sendTcpMessage(frames[0]);
            virtualIg.sendTcpMessage(frames[1]);

            for (int i = 0; i < 40 && platformSof.size() < 2; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                platform.drainIncoming();
            }

            THEN("the platform receives two separate TCP messages")
            {
                REQUIRE(platformSof.size() == 2);
                REQUIRE(platformSof[0] == 20);
                REQUIRE(platformSof[1] == 21);
            }
        }
    }
}

// =============================================================================
// viewhost 的天气 TCP 与转发来的平台命令是两条消息（平台同步设计.md §12，码 PLT-splice-cmd）。
// 两条的 TimeStampValid 都是 false。天气在第二条。
// =============================================================================
SCENARIO("viewhost TCP weather is a separate message from the relayed platform command",
         "[acceptance][platform][PLT-splice-cmd]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 37200));
        REQUIRE(linkHostIg(viewhost, realIg, 37400));

        std::vector<std::uint32_t> igFrames;
        std::vector<bool> igTimeStampValid;
        bool gotWeather = false;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            igFrames.push_back(ctrl.GetFrameCntr());
            igTimeStampValid.push_back(ctrl.GetTimeStampValid());
        });
        realIg.addCallback<CigiWeatherCtrlV4>([&](const CigiWeatherCtrlV4&) { gotWeather = true; });

        WHEN("the relay forwards a platform TCP command and viewhost then flushes its own weather TCP")
        {
            {
                auto& tcp = platform.outMsgWithIgCtrlTcp();
                CigiSymbolTextDefV4 cmd("pass");
                tcp << cmd;
                platform.flushTcp();
            }
            const auto frames = collectFrames([&] { return virtualIg.takeIncomingTcp(); }, 1);
            REQUIRE_FALSE(frames.empty());
            viewhost.sendTcpMessage(frames.back());

            {
                auto& tcp = viewhost.outMsgWithIgCtrlTcp();
                CigiWeatherCtrlV4 weather;
                weather.SetSeverity(1);
                tcp << weather;
                viewhost.flushTcp();
            }

            for (int i = 0; i < 40 && (igFrames.size() < 2 || !gotWeather); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                realIg.drainIncoming(false);
            }

            THEN("the IG sees an independent command-plane weather message")
            {
                REQUIRE(igFrames.size() == 2);
                REQUIRE(igTimeStampValid.size() == 2);
                REQUIRE(igTimeStampValid[0] == false);
                REQUIRE(gotWeather);
                REQUIRE(igTimeStampValid[1] == false);
            }
        }
    }
}

// =============================================================================
// viewhost 的天气 TCP 不打断已转发的平台 UDP 帧号（平台同步设计.md §12，码 PLT-splice-cmd）。
// UDP 帧号保持递增，天气在单独的 TCP 消息里。丢包则跳过。
// =============================================================================
SCENARIO("viewhost TCP weather does not break relayed platform UDP frame numbers",
         "[acceptance][platform][PLT-splice-cmd]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 45000));
        REQUIRE(linkHostIg(viewhost, realIg, 45200));

        std::vector<std::uint32_t> igUdpFrames;
        bool gotWeather = false;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            if (ctrl.GetTimeStampValid())
                igUdpFrames.push_back(ctrl.GetFrameCntr());
        });
        realIg.addCallback<CigiWeatherCtrlV4>([&](const CigiWeatherCtrlV4&) { gotWeather = true; });

        WHEN("the relay forwards platform UDP IGCtrl, viewhost flushes weather TCP, then more UDP IGCtrl")
        {
            std::vector<std::vector<unsigned char>> taken;
            const bool took = retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    auto more = collectFrames([&] { return virtualIg.takeIncomingUdp(); }, 1);
                    taken.insert(taken.end(), more.begin(), more.end());
                },
                [&] { return !taken.empty(); });
            if (!took)
                SKIP("UDP datagram dropped");
            for (const auto& dgram : taken)
                viewhost.sendUdpMessage(dgram);

            {
                auto& tcp = viewhost.outMsgWithIgCtrlTcp();
                CigiWeatherCtrlV4 weather;
                weather.SetSeverity(1);
                tcp << weather;
                viewhost.flushTcp();
            }

            (void)retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    auto more = collectFrames([&] { return virtualIg.takeIncomingUdp(); }, 1);
                    for (const auto& dgram : more)
                        viewhost.sendUdpMessage(dgram);
                    drainIgUntil(realIg, [&] { return gotWeather; });
                },
                [&] { return gotWeather; });
            drainIgUntil(realIg, [&] { return gotWeather; });
            if (igUdpFrames.empty())
                SKIP("UDP datagram dropped");

            THEN("arrived UDP FrameCntr stay increasing and weather is a separate TCP message")
            {
                REQUIRE(framesIncreasing(igUdpFrames));
                REQUIRE(gotWeather);
            }
        }
    }
}

// =============================================================================
// 转发后的 IG TCP 上报保持原载荷（平台同步设计.md §12，码 PLT-tcp-return）。
// 报文号 0x3001，正文是发出的那串。
// =============================================================================
SCENARIO("relayed IG TCP report keeps the original payload",
         "[acceptance][platform][PLT-tcp-return]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 37600));
        REQUIRE(linkHostIg(viewhost, realIg, 37800));

        std::uint16_t platformMsgId = 0;
        std::string platformMsg;
        platform.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            platformMsgId = msg.GetMsgID();
            platformMsg = const_cast<CigiIGMsgV4&>(msg).GetMsg();
        });

        WHEN("the real IG reports on TCP and the relay forwards the framed bytes")
        {
            {
                auto& tcp = realIg.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3001);
                report.SetMsg("up");
                tcp << report;
                realIg.flushTcp();
            }

            const auto frames = collectFrames([&] { return viewhost.takeIncomingTcp(); }, 1);
            REQUIRE_FALSE(frames.empty());
            virtualIg.sendTcpMessage(frames.back());

            for (int i = 0; i < 40 && platformMsg.empty(); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                platform.drainIncoming();
            }

            THEN("the platform sees the IG message, not a virtual-IG rewrite")
            {
                REQUIRE(platformMsgId == 0x3001);
                REQUIRE(platformMsg == "up");
            }
        }
    }
}

// =============================================================================
// 转发后的 UDP IGCtrl 保持平台的帧号和时间戳（平台同步设计.md §12，码 PLT-filter-igctrl）。
// TimeStampValid 为 true。实 IG 只收到这一条，viewhost 没有再发 IGCtrl。
// =============================================================================
SCENARIO("relayed platform UDP IGCtrl keeps the original header",
         "[acceptance][platform][PLT-filter-igctrl]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 38000));
        REQUIRE(linkHostIg(viewhost, realIg, 38200));

        std::uint32_t igFrameCntr = 0xffffffffu;
        std::uint32_t igTimeStamp = 0;
        bool igTimeStampValid = false;
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            igFrameCntr = ctrl.GetFrameCntr();
            igTimeStamp = ctrl.GetTimeStamp();
            igTimeStampValid = ctrl.GetTimeStampValid();
        });

        WHEN("the platform sends UDP IGCtrl after two earlier frames and the relay forwards the datagram")
        {
            for (int i = 0; i < 3; ++i)
            {
                platform.outMsgWithIgCtrlUdp();
                platform.flushUdp();
            }

            const auto dgrams = collectFrames([&] { return virtualIg.takeIncomingUdp(); }, 3);
            REQUIRE(dgrams.size() >= 3);
            const auto& last = dgrams.back();
            CigiIGCtrlV4 sent;
            REQUIRE(sent.Unpack(const_cast<unsigned char*>(last.data()), false, nullptr) >= 0);
            viewhost.sendUdpMessage(last);

            for (int i = 0; i < 40 && igFrameCntr == 0xffffffffu; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                realIg.drainIncoming(false);
            }

            THEN("the real IG sees the platform data-plane FrameCntr and TimeStamp, and viewhost did not add another IGCtrl")
            {
                REQUIRE(igFrameCntr == sent.GetFrameCntr());
                REQUIRE(igTimeStamp == sent.GetTimeStamp());
                REQUIRE(igTimeStampValid);
                REQUIRE(realIg.igCtrlReceivedCount() == 1);
            }
        }
    }
}

// =============================================================================
// 虚 IG 回给平台的 SOF 与平台帧号一致，实 IG 的 SOF 留在 viewhost（平台同步设计.md §12，码 PLT-relay-sof）。
// 两边的 SOF 计数都是 1。
// =============================================================================
SCENARIO("virtual IG packSof after UDP forward is the only SOF the platform sees",
         "[acceptance][platform][PLT-relay-sof]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 38400));
        REQUIRE(linkHostIg(viewhost, realIg, 38600));

        std::uint32_t platformSof = 0xffffffffu;
        platform.addCallback<CigiSOFV4>([&](const CigiSOFV4& sof) { platformSof = sof.GetFrameCntr(); });

        WHEN("the relay forwards one platform UDP IGCtrl, echos SOF from the virtual IG, and the real IG replies SOF to viewhost")
        {
            for (int i = 0; i < 3; ++i)
            {
                platform.outMsgWithIgCtrlUdp();
                platform.flushUdp();
            }
            const auto dgrams = collectFrames([&] { return virtualIg.takeIncomingUdp(); }, 3);
            REQUIRE(dgrams.size() >= 3);
            const auto& last = dgrams.back();
            CigiIGCtrlV4 sent;
            REQUIRE(sent.Unpack(const_cast<unsigned char*>(last.data()), false, nullptr) >= 0);
            viewhost.sendUdpMessage(last);
            virtualIg.sendSofForIgCtrl(last);

            for (int i = 0; i < 40 && realIg.igCtrlReceivedCount() == 0; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                realIg.drainIncoming(true);
            }
            for (int i = 0; i < 40 && platform.sofReceivedCount() == 0; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                platform.drainIncoming();
            }
            for (int i = 0; i < 40 && viewhost.sofReceivedCount() == 0; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                viewhost.drainIncoming();
            }

            THEN("the platform receives one matching SOF from the virtual IG, not the real IG SOF")
            {
                REQUIRE(platform.sofReceivedCount() == 1);
                REQUIRE(platformSof == sent.GetFrameCntr());
                REQUIRE(viewhost.sofReceivedCount() == 1);
            }
        }
    }
}

// =============================================================================
// 没有实 IG 可收 UDP 时，虚 IG 仍按平台帧号回 SOF（平台同步设计.md §12，码 PLT-relay-sof）。
// 丢包则跳过。
// =============================================================================
SCENARIO("virtual IG packSof when no real IG is ready to receive UDP",
         "[acceptance][platform][PLT-relay-sof]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host with no ready IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        REQUIRE(linkHostIg(platform, virtualIg, 45400));
        REQUIRE(viewhost.initialize(makeTestHostConfig(45600)));

        std::uint32_t platformSof = 0xffffffffu;
        platform.addCallback<CigiSOFV4>([&](const CigiSOFV4& sof) { platformSof = sof.GetFrameCntr(); });

        WHEN("the relay takes a platform UDP IGCtrl with no real IG to send to, and the virtual IG echos SOF")
        {
            std::vector<unsigned char> last;
            CigiIGCtrlV4 sent;
            const bool took = retryUdpUntil(
                [&] {
                    platform.outMsgWithIgCtrlUdp();
                    platform.flushUdp();
                    const auto dgrams = collectFrames([&] { return virtualIg.takeIncomingUdp(); }, 1);
                    if (dgrams.empty())
                        return;
                    last = dgrams.back();
                    (void)sent.Unpack(const_cast<unsigned char*>(last.data()), false, nullptr);
                },
                [&] { return !last.empty(); });
            if (!took)
                SKIP("UDP datagram dropped");
            REQUIRE(sent.Unpack(const_cast<unsigned char*>(last.data()), false, nullptr) >= 0);
            viewhost.sendUdpMessage(last);
            const bool sofed = retryUdpUntil(
                [&] {
                    virtualIg.sendSofForIgCtrl(last);
                    drainHostUntil(platform, [&] { return platform.sofReceivedCount() > 0; });
                },
                [&] { return platform.sofReceivedCount() > 0; });
            if (!sofed)
                SKIP("UDP datagram dropped");

            THEN("the platform receives a matching SOF")
            {
                REQUIRE(platform.sofReceivedCount() >= 1);
                REQUIRE(platformSof == sent.GetFrameCntr());
            }
        }
    }
}

// =============================================================================
// 多条平台 TCP 和 UDP 都按序到达实 IG（平台同步设计.md §12，码 PLT-multi-down）。
// 两条文本是 tcp-0、tcp-1，两条 UDP 帧号是 0、1。
// =============================================================================
SCENARIO("relay forwards multiple platform TCP and UDP messages to the IG",
         "[acceptance][platform][PLT-multi-down]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 38800));
        REQUIRE(linkHostIg(viewhost, realIg, 39000));

        std::vector<std::string> igTexts;
        std::vector<std::uint32_t> igUdpFrames;
        realIg.addCallback<CigiSymbolTextDefV4>([&](const CigiSymbolTextDefV4& txt) {
            igTexts.push_back(const_cast<CigiSymbolTextDefV4&>(txt).GetText());
        });
        realIg.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) {
            if (ctrl.GetTimeStampValid())
                igUdpFrames.push_back(ctrl.GetFrameCntr());
        });

        WHEN("the platform sends two TCP commands and two UDP IGCtrl and the relay forwards each")
        {
            auto sendTcpText = [&](const char* text) {
                auto& tcp = platform.outMsgWithIgCtrlTcp();
                CigiSymbolTextDefV4 cmd(text);
                tcp << cmd;
                platform.flushTcp();
            };
            sendTcpText("tcp-0");
            sendTcpText("tcp-1");
            platform.outMsgWithIgCtrlUdp();
            platform.flushUdp();
            platform.outMsgWithIgCtrlUdp();
            platform.flushUdp();

            const auto tcpFrames = collectFrames([&] { return virtualIg.takeIncomingTcp(); }, 2);
            REQUIRE(tcpFrames.size() >= 2);
            viewhost.sendTcpMessage(tcpFrames[0]);
            viewhost.sendTcpMessage(tcpFrames[1]);

            const auto udpFrames = collectFrames([&] { return virtualIg.takeIncomingUdp(); }, 2);
            REQUIRE(udpFrames.size() >= 2);
            viewhost.sendUdpMessage(udpFrames[0]);
            viewhost.sendUdpMessage(udpFrames[1]);

            for (int i = 0; i < 40 && (igTexts.size() < 2 || igUdpFrames.size() < 2); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                realIg.drainIncoming(false);
            }

            THEN("the real IG receives every TCP payload and every UDP data-plane IGCtrl in order")
            {
                REQUIRE(igTexts.size() == 2);
                REQUIRE(igTexts[0] == "tcp-0");
                REQUIRE(igTexts[1] == "tcp-1");
                REQUIRE(igUdpFrames.size() == 2);
                REQUIRE(igUdpFrames[0] == 0);
                REQUIRE(igUdpFrames[1] == 1);
            }
        }
    }
}

// =============================================================================
// 多条 IG TCP 上报到达平台，实 IG 的 UDP SOF 留在 viewhost（平台同步设计.md §12，码 PLT-multi-up）。
// 平台 SOF 计数停在转发 TCP 之后的值。
// =============================================================================
SCENARIO("relay forwards multiple IG TCP reports; IG UDP SOF stays on viewhost",
         "[acceptance][platform][PLT-multi-up]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host linked to a real IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        IgSync realIg;
        REQUIRE(linkHostIg(platform, virtualIg, 39200));
        REQUIRE(linkHostIg(viewhost, realIg, 39400));

        std::vector<std::pair<std::uint16_t, std::string>> platformMsgs;
        platform.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            platformMsgs.emplace_back(msg.GetMsgID(), const_cast<CigiIGMsgV4&>(msg).GetMsg());
        });

        WHEN("the real IG sends two TCP reports and two UDP SOF, and the relay forwards only TCP")
        {
            auto sendIgTcp = [&](std::uint16_t msgId, const char* text) {
                auto& tcp = realIg.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(msgId);
                report.SetMsg(text);
                tcp << report;
                realIg.flushTcp();
            };
            sendIgTcp(0x3001, "up-0");
            sendIgTcp(0x3002, "up-1");

            const auto tcpFrames = collectFrames([&] { return viewhost.takeIncomingTcp(); }, 2);
            REQUIRE(tcpFrames.size() >= 2);
            virtualIg.sendTcpMessage(tcpFrames[0]);
            virtualIg.sendTcpMessage(tcpFrames[1]);

            for (int i = 0; i < 40 && platformMsgs.size() < 2; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                platform.drainIncoming();
            }
            const auto platformSofAfterTcp = platform.sofReceivedCount();

            realIg.outMsgWithSofUdp();
            realIg.flushUdp();
            realIg.outMsgWithSofUdp();
            realIg.flushUdp();
            for (int i = 0; i < 40 && viewhost.sofReceivedCount() < 2; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                viewhost.drainIncoming();
            }
            platform.drainIncoming();

            THEN("the platform receives both TCP reports and none of the real IG UDP SOF")
            {
                REQUIRE(platformMsgs.size() == 2);
                REQUIRE(platformMsgs[0].first == 0x3001);
                REQUIRE(platformMsgs[0].second == "up-0");
                REQUIRE(platformMsgs[1].first == 0x3002);
                REQUIRE(platformMsgs[1].second == "up-1");
                REQUIRE(viewhost.sofReceivedCount() == 2);
                REQUIRE(platform.sofReceivedCount() == platformSofAfterTcp);
            }
        }
    }
}

// =============================================================================
// 还没有平台 ownship 时，拒绝停止转发（平台同步设计.md §12，码 PLT-relay-pause）。
// 拒绝之后，随后的平台 ownship 仍能到达。viewhost 不发数据面 IGCtrl。ownship 丢包则跳过。
// =============================================================================
SCENARIO("viewhost refuses to stop forwarding before a platform ownship has arrived",
         "[acceptance][platform][PLT-relay-pause]")
{
    GIVEN("a linked relay that has only forwarded a platform IGCtrl without ownship")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 51200, 51400));
        retryUdpUntil(
            [&] {
                hostSendFrame(platform);
                viewhost.pollRelay();
                drainIgUntil(realIg, [&] { return realIg.igCtrlReceivedCount() > 0; });
            },
            [&] { return realIg.igCtrlReceivedCount() > 0; });

        WHEN("the operator turns off forwarding")
        {
            const bool paused = viewhost.setRelayForwarding(false);

            THEN("the stop is refused")
            {
                REQUIRE_FALSE(paused);
            }

            AND_WHEN("the platform later sends ownship")
            {
                OwnshipEyeCapture cap;
                captureOwnship(realIg, cap);
                const auto platformEye = makeEye(31.2, 121.5, 80.0, 45.0, 5.0);
                const bool observed = retryUdpUntil(
                    [&] {
                        hostSendEyePose(platform, platformEye);
                        viewhost.pollRelay();
                        drainIgUntil(realIg, [&] { return cap.got; });
                    },
                    [&] { return cap.got; });
                if (!observed)
                    SKIP("UDP datagram dropped");

                THEN("the IG receives that platform ownship")
                {
                    REQUIRE(cap.lat == Catch::Approx(platformEye.x));
                    REQUIRE(cap.lon == Catch::Approx(platformEye.y));
                    REQUIRE(cap.alt == Catch::Approx(platformEye.z));
                }
            }

            AND_WHEN("viewhost updates with a keyboard eye")
            {
                const auto sentBefore = viewhost.igCtrlSentCount();
                const auto keyboard = makeEye(1.0, 2.0, 3.0, 10.0, 20.0);
                viewhost.update(&keyboard);

                THEN("viewhost sends no data-plane IGCtrl")
                {
                    REQUIRE(viewhost.igCtrlSentCount() == sentBefore);
                }
            }
        }
    }
}

// =============================================================================
// 停止转发后，双向都不再放行，缓存眼点停在已到达的平台 ownship（平台同步设计.md §12，码 PLT-relay-pause）。
// 停转期间的平台命令、新眼点和 IG 上报都到不了对端。viewhost 再发两次当前眼点，IG 两次收到的都是这条平台 ownship。丢包则跳过。
// =============================================================================
SCENARIO("stopped forwarding blocks both directions and repeats the last platform eye",
         "[acceptance][platform][PLT-relay-pause]")
{
    GIVEN("a linked relay that has forwarded a platform ownship")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        const auto platformEye = makeEye(31.2, 121.5, 80.0, 45.0, 5.0);
        const auto laterEye = makeEye(40.0, 116.0, 200.0, 90.0, 0.0);
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 51600, 51800));
        if (!relayPlatformOwnship(platform, viewhost, realIg, platformEye))
            SKIP("UDP datagram dropped");

        WHEN("the operator turns off forwarding")
        {
            const bool paused = viewhost.setRelayForwarding(false);
            const auto cached = viewhost.relayEye();

            THEN("forwarding is off and the current eye is that platform ownship")
            {
                REQUIRE(paused);
                REQUIRE_FALSE(viewhost.relayForwarding());
                REQUIRE(cached.has_value());
                REQUIRE(cached->x == Catch::Approx(platformEye.x));
                REQUIRE(cached->y == Catch::Approx(platformEye.y));
                REQUIRE(cached->z == Catch::Approx(platformEye.z));
                REQUIRE(cached->yawDeg == Catch::Approx(platformEye.yawDeg));
                REQUIRE(cached->pitchDeg == Catch::Approx(platformEye.pitchDeg));
            }

            AND_WHEN("the platform sends a command and a new ownship, and the IG reports")
            {
                std::string igText;
                realIg.addCallback<CigiSymbolTextDefV4>([&](const CigiSymbolTextDefV4& txt) {
                    igText = const_cast<CigiSymbolTextDefV4&>(txt).GetText();
                });
                OwnshipEyeCapture later;
                captureOwnship(realIg, later);
                std::string platformMsg;
                platform.addCallback<CigiIGMsgV4>(
                    [&](const CigiIGMsgV4& msg) { platformMsg = const_cast<CigiIGMsgV4&>(msg).GetMsg(); });
                sendTcpSymbol(platform, "late");
                sendIgTcpReport(realIg, 0x3001, "paused");
                for (int i = 0; i < 5; ++i)
                {
                    hostSendEyePose(platform, laterEye);
                    viewhost.pollRelay();
                    realIg.drainIncoming(false);
                    platform.drainIncoming();
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }

                THEN("neither side receives those messages")
                {
                    REQUIRE(igText.empty());
                    REQUIRE_FALSE(later.got);
                    REQUIRE(platformMsg.empty());
                }
            }

            AND_WHEN("viewhost sends the current eye twice")
            {
                REQUIRE(cached.has_value());
                std::vector<OwnshipEyeCapture> frames;
                realIg.addCallback<CigiEntityPositionCtrlV4>([&](const CigiEntityPositionCtrlV4& pose) {
                    if (pose.GetEntityID() != 0)
                        return;
                    frames.push_back(ownshipFrom(pose));
                });
                const auto eye = *cached;
                const bool observed = retryUdpUntil(
                    [&] {
                        frames.clear();
                        viewhost.update(&eye);
                        drainIgUntil(realIg, [&] { return !frames.empty(); });
                        viewhost.update(&eye);
                        drainIgUntil(realIg, [&] { return frames.size() >= 2; });
                    },
                    [&] { return frames.size() >= 2; });
                if (!observed)
                    SKIP("UDP datagram dropped");

                THEN("both IG ownships are the last platform eye")
                {
                    REQUIRE(frames.size() >= 2);
                    requireOwnship(frames[0], platformEye);
                    requireOwnship(frames[1], platformEye);
                }
            }
        }
    }
}

// =============================================================================
// 恢复转发后，丢掉停转期间排队的命令，并采用下一条平台眼点（平台同步设计.md §12，码 PLT-relay-resume）。
// 恢复后立刻 tick，排队命令不到。新眼点到达后，viewhost 仍不发数据面 IGCtrl。丢包则跳过。
// =============================================================================
SCENARIO("resumed forwarding drops paused traffic and applies the next platform eye",
         "[acceptance][platform][PLT-relay-resume]")
{
    GIVEN("a linked relay that stopped forwarding after a platform ownship, with a command queued while stopped")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        const auto platformEye = makeEye(31.2, 121.5, 80.0, 45.0, 5.0);
        const auto keyboard = makeEye(32.0, 122.0, 90.0, 12.0, 3.0);
        const auto jumpEye = makeEye(40.0, 116.0, 200.0, 90.0, 0.0);
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 52000, 52200));
        if (!relayPlatformOwnship(platform, viewhost, realIg, platformEye))
            SKIP("UDP datagram dropped");
        REQUIRE(viewhost.setRelayForwarding(false));
        sendTcpSymbol(platform, "queued");

        std::string igText;
        realIg.addCallback<CigiSymbolTextDefV4>(
            [&](const CigiSymbolTextDefV4& txt) { igText = const_cast<CigiSymbolTextDefV4&>(txt).GetText(); });

        WHEN("forwarding resumes and the relay ticks without a new platform command")
        {
            REQUIRE(viewhost.setRelayForwarding(true));
            tickRelay(viewhost);
            drainIgUntil(realIg, [&] { return !igText.empty(); });

            THEN("the IG does not receive the queued command")
            {
                REQUIRE(igText.empty());
            }

            AND_WHEN("the platform sends a new ownship and viewhost updates a local eye")
            {
                OwnshipEyeCapture later;
                captureOwnship(realIg, later);
                const auto sentBefore = viewhost.igCtrlSentCount();
                const bool observed = retryUdpUntil(
                    [&] {
                        hostSendEyePose(platform, jumpEye);
                        viewhost.pollRelay();
                        viewhost.update(&keyboard);
                        drainIgUntil(realIg, [&] { return later.got; });
                    },
                    [&] { return later.got; });
                if (!observed)
                    SKIP("UDP datagram dropped");

                THEN("the IG ownship is that platform eye and viewhost sends no data-plane IGCtrl")
                {
                    requireOwnship(later, jumpEye);
                    REQUIRE(viewhost.igCtrlSentCount() == sentBefore);
                }
            }
        }
    }
}
