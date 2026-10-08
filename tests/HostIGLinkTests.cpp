#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/CigiIncludes.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/HostDriver.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SyncInterface.h>
#include <aerovista/sync/SynchronSystem.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"
#include "CigiIGSession.h"
#include "CigiSOFV4.h"
#include "CigiSymbolTextDefV4.h"

#include "CigiCollDetSegDefV4.h"
#include "CigiCollDetSegRespV4.h"
#include "CigiIGMsgV4.h"
#include "CigiWeatherCtrlV4.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "Common.h"

using aerovista::sync::HostConfig;
using aerovista::sync::HostDriver;
using aerovista::sync::HostStatus;
using aerovista::sync::HostSync;
using aerovista::sync::IgConfig;
using aerovista::sync::IgStatus;
using aerovista::sync::IgSync;
using aerovista::sync::OffsetDeg;
using aerovista::sync::SynchronSystem;
using aerovista::sync::SyncInterface;
using aerovista::sync::SyncSystemConfig;
using aerovista::sync::TcpSocket;
namespace cigi_wire = aerovista::sync::cigi_wire;

// 协议分层（测试约定）：
// - 握手：CIGI HELLO（SOF+IGMsg，无 TCP ACK）+ UDP_SYNC / ACK —— §1 / PLT-hello-ch。
// - 数据面（帧节拍 / 眼点 / SOF）：CIGI V4 CCL —— IGCtrl (+ 可选 EntityPositionCtrl) / SOF。
//   数据面契约走 HostSync/IgSync 可观察收发（[wire-contract]）；CCL 首包约束仍为 session 反向用例单测。


namespace
{
    // 默认端口见 doc/design/多通道同步/多通道同步模块设计.md
    HostConfig makeHostLocal()
    {
        return HostConfig{8000, 8100};
    }

    IgConfig makeIgLocal(int udpRecvPort = 8001)
    {
        return IgConfig{udpRecvPort, {"127.0.0.1", 8100, 8000}};
    }

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
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

    bool waitVirtualIgLinked(HostDriver& viewhost, HostSync& platform)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
        while ((!viewhost.virtualIgLinked() || platform.readyIgCount() != 1) &&
               std::chrono::steady_clock::now() < deadline)
        {
            viewhost.pollRelay();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return viewhost.virtualIgLinked() && platform.readyIgCount() == 1;
    }

    bool startHostDriverRelay(HostSync& platform, HostDriver& viewhost, IgSync& realIg, int platformBase,
                              int viewhostBase)
    {
        if (!platform.initialize(makeTestHostConfig(platformBase)))
            return false;
        platform.run();
        if (!viewhost.initialize(makeRelayViewhostConfig(viewhostBase, platformBase, 1)))
            return false;
        const IgConfig realCfg = makeTestIgConfig(viewhostBase + 1, viewhostBase);
        if (!realIg.initialize(realCfg.udpPortRecv, 0))
            return false;
        if (!realIg.connect(realCfg.target))
            return false;
        return waitVirtualIgLinked(viewhost, platform);
    }

    bool startHostDriverRelayTwoIgs(HostSync& platform, HostDriver& viewhost, IgSync& master, IgSync& side,
                                    int platformBase, int viewhostBase)
    {
        if (!platform.initialize(makeTestHostConfig(platformBase)))
            return false;
        platform.run();
        if (!viewhost.initialize(makeRelayViewhostConfig(viewhostBase, platformBase, 2)))
            return false;
        const IgConfig masterCfg = makeTestIgConfig(viewhostBase + 1, viewhostBase);
        const IgConfig sideCfg = makeTestIgConfig(viewhostBase + 3, viewhostBase);
        if (!master.initialize(masterCfg.udpPortRecv, 0))
            return false;
        if (!side.initialize(sideCfg.udpPortRecv, 1))
            return false;
        if (!master.connect(masterCfg.target))
            return false;
        if (!side.connect(sideCfg.target))
            return false;
        return waitVirtualIgLinked(viewhost, platform);
    }

    bool startHostTwoIgs(HostSync& host, IgSync& master, IgSync& side, int base)
    {
        if (!host.initialize(makeTestHostConfig(base)))
            return false;
        const IgConfig masterCfg = makeTestIgConfig(base + 1, base);
        const IgConfig sideCfg = makeTestIgConfig(base + 3, base);
        if (!master.initialize(masterCfg.udpPortRecv, 0))
            return false;
        if (!side.initialize(sideCfg.udpPortRecv, 1))
            return false;
        if (!master.connect(masterCfg.target))
            return false;
        if (!side.connect(sideCfg.target))
            return false;
        return host.readyIgCount() == 2;
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

    bool framesIncreasing(const std::vector<std::uint32_t>& frames)
    {
        for (std::size_t i = 1; i < frames.size(); ++i)
        {
            if (frames[i] <= frames[i - 1])
                return false;
        }
        return true;
    }

    // UDP 可丢：actual 落在 [expected-slack, expected]
    bool approxAtMost(std::uint32_t actual, int expected, int slack)
    {
        const auto exp = static_cast<std::uint32_t>(expected);
        const auto minOk = exp > static_cast<std::uint32_t>(slack) ? exp - static_cast<std::uint32_t>(slack) : 0u;
        return actual >= minOk && actual <= exp;
    }

    /// IGCtrl 由 outMsgWithIgCtrlUdp() 自动前置（帧号/自计时时间戳）；hostSendFrame 只发无眼点帧。
    void hostSendFrame(HostSync& host, double /*simTimeMs*/)
    {
        host.outMsgWithIgCtrlUdp();
        host.flushUdp();
    }

    void hostSendEyePose(HostSync& host, const cigi_wire::EyePose& eye)
    {
        auto& omsg = host.outMsgWithIgCtrlUdp();
        cigi_wire::appendEye(omsg, &eye);
        host.flushUdp();
    }


    bool linkHostIg(HostSync& host, IgSync& ig, int base)
    {
        if (!host.initialize(makeTestHostConfig(base)))
            return false;
        const IgConfig igConfig = makeTestIgConfig(base + 1, base);
        if (!ig.initialize(igConfig.udpPortRecv) || !ig.connect(igConfig.target))
            return false;
        host.run();
        return true;
    }

    std::vector<std::vector<unsigned char>> collectFrames(
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

    std::vector<unsigned char> concatFrames(const std::vector<unsigned char>& a,
                                            const std::vector<unsigned char>& b)
    {
        std::vector<unsigned char> out = a;
        out.insert(out.end(), b.begin(), b.end());
        return out;
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

    void captureOwnship(IgSync& ig, OwnshipEyeCapture& cap)
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

    bool seedPlatformOwnship(HostSync& platform, HostDriver& viewhost, IgSync& ig, const cigi_wire::EyePose& eye)
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

    void pumpIgCtrlFrames(HostSync& host, IgSync& ig, int frames = 5)
    {
        for (int i = 0; i < frames; ++i)
        {
            hostSendFrame(host, i * 16.667);
            ig.drainIncoming();
            ig.update();
        }
    }

    void pumpOwnshipEyeFrames(HostSync& host, IgSync& ig, const cigi_wire::EyePose& eye, int frames = 5)
    {
        for (int i = 0; i < frames; ++i)
        {
            hostSendEyePose(host, eye);
            ig.drainIncoming();
            ig.update();
        }
    }

    // 独立 Host 端点（测试用）：持 HostSync，RAII 生命周期。
    // 端口语义 = makeTestHostConfig（Common.h），与 makeTestIgConfig 的 target 对齐。
    struct TestHost
    {
        HostSync sync;
        bool init(int base)
        {
            if (!sync.initialize(makeTestHostConfig(base)))
                return false;
            sync.run();
            return true;
        }
    };
} // namespace

SCENARIO("linked IG receives Host ownship eye as Detach LLA EntityID 0",
         "[integration][sync][cigi][wire-contract][lla][CIGI-ownship-lla]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(linkHostIg(host, ig, 19100));

        OwnshipEyeCapture cap;
        captureOwnship(ig, cap);

        WHEN("Host sends IGCtrl with an ownship eye")
        {
            cigi_wire::EyePose eye{};
            eye.x = 39.9;
            eye.y = 116.4;
            eye.z = 500.0;
            eye.yawDeg = 30.0;
            eye.pitchDeg = 10.0;
            pumpOwnshipEyeFrames(host, ig, eye);

            THEN("IG unpacks Detach LLA ownship at EntityID 0 ParentID 0")
            {
                REQUIRE(ig.igCtrlReceivedCount() >= 1);
                REQUIRE(cap.got);
                REQUIRE(cap.entityId == 0);
                REQUIRE(cap.parentId == 0);
                REQUIRE(cap.attachState == CigiBaseEntityPositionCtrl::Detach);
                REQUIRE(cap.lat == Catch::Approx(eye.x));
                REQUIRE(cap.lon == Catch::Approx(eye.y));
                REQUIRE(cap.alt == Catch::Approx(eye.z));
                REQUIRE(cap.yawDeg == Catch::Approx(eye.yawDeg));
                REQUIRE(cap.pitchDeg == Catch::Approx(eye.pitchDeg));
            }
        }
    }
}

SCENARIO("linked IG receives IGCtrl when Host sends a frame without eye",
         "[integration][sync][cigi][wire-contract][CIGI-igctrl-no-eye]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(linkHostIg(host, ig, 19200));

        OwnshipEyeCapture cap;
        captureOwnship(ig, cap);

        WHEN("Host sends IGCtrl with no ownship eye")
        {
            pumpIgCtrlFrames(host, ig);

            THEN("IG received IGCtrl and no ownship EntityPosition")
            {
                REQUIRE(ig.igCtrlReceivedCount() >= 1);
                REQUIRE_FALSE(cap.got);
            }
        }
    }
}

namespace
{
    // 手工构造一条「首包非 IGCtrl/SOF」的最小 CIGI4 消息（绕过 Pack 的版本域歧义）。
    // 布局：PacketSize(2,LE) + PacketID(2,LE) + 版本域(2) + 保留(2)。
    std::vector<unsigned char> makeBadFirstPacketMsg(std::uint16_t packetId)
    {
        std::vector<unsigned char> msg(8, 0);
        msg[0] = 8; // PacketSize=8（LE）
        msg[1] = 0;
        msg[2] = static_cast<unsigned char>(packetId & 0xFF); // PacketID
        msg[3] = static_cast<unsigned char>(packetId >> 8);
        msg[4] = 4; // CIGI 主版本 4（CIGI4 版本域在字节 4）
        return msg;
    }
} // namespace

TEST_CASE("CIGI IG rejects a message whose first packet is not IGCtrl",
          "[unit][cigi][wire-contract][negative][CIGI-first-igctrl]")
{
    // CCL 硬约束（CigiIncomingMsg.cpp CheckFirstPacket）：IG 入站消息首包必须 IGCtrl（0x0000），
    // 否则拒绝。构造首包 PacketID=0x0001（EntityPosition，非 IGCtrl）。
    const auto badMsg = makeBadFirstPacketMsg(0x0001);

    auto ig = std::make_unique<CigiIGSession>(1, 4096, 1, 4096);
    int stat = CIGI_SUCCESS;
    try
    {
        stat = ig->GetIncomingMsgMgr().ProcessIncomingMsg(
            const_cast<unsigned char*>(badMsg.data()), static_cast<int>(badMsg.size()));
    }
    catch (...)
    {
        stat = CIGI_ERROR_MISSING_IG_CONTROL_PACKET; // 抛异常也视为拒绝
    }
    REQUIRE(stat != CIGI_SUCCESS);
}

TEST_CASE("CIGI Host rejects a message whose first packet is not SOF",
          "[unit][cigi][wire-contract][negative][CIGI-first-sof]")
{
    // CCL 硬约束（CigiIncomingMsg.cpp CheckFirstPacket）：Host 入站消息首包必须 SOF（0xffff），
    // 否则拒绝。构造首包 PacketID=0x0001（IGCtrl，非 SOF）。
    const auto badMsg = makeBadFirstPacketMsg(0x0001);

    auto host = std::make_unique<CigiHostSession>(1, 4096, 1, 4096);
    int stat = CIGI_SUCCESS;
    try
    {
        stat = host->GetIncomingMsgMgr().ProcessIncomingMsg(
            const_cast<unsigned char*>(badMsg.data()), static_cast<int>(badMsg.size()));
    }
    catch (...)
    {
        stat = CIGI_ERROR_MISSING_SOF_PACKET; // 抛异常也视为拒绝
    }
    REQUIRE(stat != CIGI_SUCCESS);
}

// =============================================================================
// 1. 连接面（集成；握手为 CIGI HELLO + UDP_SYNC）
// =============================================================================

SCENARIO("Host initializes with no ready IG", "[integration][sync][initialize][HS-host-init-empty]")
{
    GIVEN("a new HostSync")
    {
        HostSync host;

        WHEN("it is initialized")
        {
            const bool ok = host.initialize(makeHostLocal());

            THEN("initialization succeeds and no IG is ready yet")
            {
                REQUIRE(ok);
                REQUIRE_FALSE(host.hasReadyIg());
                REQUIRE(host.readyIgCount() == 0);
            }
        }
    }
}

SCENARIO("IG initializes disconnected from any Host", "[integration][sync][initialize][HS-ig-init-disconnected]")
{
    GIVEN("a new IgSync")
    {
        IgSync ig;

        WHEN("it is initialized")
        {
            const bool ok = ig.initialize(makeIgLocal().udpPortRecv);

            THEN("initialization succeeds and it is not connected to a Host yet")
            {
                REQUIRE(ok);
                REQUIRE_FALSE(ig.tcpConnected());
                REQUIRE_FALSE(ig.udpSynced());
            }
        }
    }
}

SCENARIO("IG connect fails when Host is not running", "[integration][sync][connect][failure][HS-connect-fail-no-host]")
{
    GIVEN("an IG initialized without a running Host")
    {
        IgSync ig;
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));

        WHEN("the IG connects to a Host endpoint")
        {
            const bool connected = ig.connect(makeIgLocal().target);

            THEN("connect fails and neither plane reports connected")
            {
                REQUIRE_FALSE(connected);
                REQUIRE_FALSE(ig.tcpConnected());
                REQUIRE_FALSE(ig.udpSynced());
            }
        }
    }
}

SCENARIO("IG connects successfully when Host is already waiting", "[integration][sync][connect][HS-connect-ok]")
{
    GIVEN("a Host that has been initialized and is waiting for IGs")
    {
        HostSync host;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE_FALSE(host.hasReadyIg());

        AND_GIVEN("an IG that has been initialized")
        {
            IgSync ig;
            REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));

            WHEN("the IG connects to the Host endpoint")
            {
                const bool connected = ig.connect(makeIgLocal().target);

                THEN("both planes are synced and Host has one ready IG")
                {
                    REQUIRE(connected);
                    REQUIRE(ig.tcpConnected());
                    REQUIRE(ig.udpSynced());
                    REQUIRE(host.hasReadyIg());
                    REQUIRE(host.readyIgCount() == 1);
                }
            }
        }
    }
}

SCENARIO("IG disconnects when Host goes offline", "[integration][sync][connect][disconnect][HS-disconnect-host-offline]")
{
    GIVEN("a connected Host and IG")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));
        REQUIRE(ig.connect(makeIgLocal().target));
        REQUIRE(ig.tcpConnected());
        REQUIRE(ig.udpSynced());

        WHEN("the Host goes offline")
        {
            host.shutdown();

            THEN("IG reports disconnected on both planes")
            {
                REQUIRE_FALSE(ig.tcpConnected());
                REQUIRE_FALSE(ig.udpSynced());
            }
        }
    }
}

SCENARIO("IG connect fails when UDP peer ports are wrong but TCP port is valid",
         "[integration][sync][connect][failure][HS-connect-fail-udp-port]")
{
    GIVEN("a Host waiting for IGs")
    {
        HostSync host;
        REQUIRE(host.initialize(makeHostLocal()));

        AND_GIVEN("an IG initialized with correct local ports")
        {
            IgSync ig;
            REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));

            WHEN("the IG connects using a Host target with wrong UDP ports")
            {
                // Connect 使用 HostTarget::udpPortRecv 作为 Host UDP 收端口。
                IgConfig badUdpConfig{8001, {"127.0.0.1", 8100, 9999}};
                const bool connected = ig.connect(badUdpConfig.target);

                THEN("overall connect fails and neither plane is ready")
                {
                    REQUIRE_FALSE(connected);
                    REQUIRE_FALSE(ig.tcpConnected());
                    REQUIRE_FALSE(ig.udpSynced());
                    REQUIRE_FALSE(host.hasReadyIg());
                }
            }
        }
    }
}

SCENARIO("Host accepts multiple co-located IG connections", "[integration][sync][connect][multi-ig][HS-multi-ig-ready]")
{
    GIVEN("a Host waiting for IGs")
    {
        HostSync host;
        REQUIRE(host.initialize(makeHostLocal()));

        WHEN("two co-located IGs initialize on distinct UDP recv ports and connect")
        {
            IgSync ig1;
            IgSync ig2;
            REQUIRE(ig1.initialize(makeIgLocal(8001).udpPortRecv, 0));
            REQUIRE(ig2.initialize(makeIgLocal(8003).udpPortRecv, 1));

            REQUIRE(ig1.connect(makeIgLocal(8001).target));
            REQUIRE(ig2.connect(makeIgLocal(8003).target));

            THEN("both IGs are synced and Host reports two ready IGs")
            {
                REQUIRE(ig1.tcpConnected());
                REQUIRE(ig1.udpSynced());
                REQUIRE(ig2.tcpConnected());
                REQUIRE(ig2.udpSynced());
                REQUIRE(host.readyIgCount() == 2);
            }
        }
    }
}

SCENARIO("Host snapshot records the channelId from IG HELLO",
         "[acceptance][bdd][platform][PLT-hello-ch]")
{
    GIVEN("a Host waiting for IGs")
    {
        HostSync host;
        REQUIRE(host.initialize(makeHostLocal()));

        AND_GIVEN("an IG initialized as channel 2")
        {
            IgSync ig;
            REQUIRE(ig.initialize(makeIgLocal().udpPortRecv, 2));

            WHEN("the IG connects")
            {
                REQUIRE(ig.connect(makeIgLocal().target));

                THEN("igSnapshot carries channelId 2")
                {
                    REQUIRE(host.readyIgCount() == 1);
                    const auto peers = host.igSnapshot();
                    REQUIRE(peers.size() == 1);
                    REQUIRE(peers.front().channelId == 2);
                }
            }
        }
    }
}

SCENARIO("Host rejects a later IG HELLO that reuses channelId",
         "[acceptance][bdd][platform][PLT-hello-ch]")
{
    GIVEN("a Host with one ready IG on channel 1")
    {
        HostSync host;
        IgSync first;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(first.initialize(makeIgLocal(8001).udpPortRecv, 1));
        REQUIRE(first.connect(makeIgLocal(8001).target));
        REQUIRE(host.readyIgCount() == 1);

        WHEN("a second IG connects with the same channelId")
        {
            IgSync second;
            REQUIRE(second.initialize(makeIgLocal(8003).udpPortRecv, 1));
            const bool connected = second.connect(makeIgLocal(8003).target);

            THEN("the later HELLO is refused and the first peer stays")
            {
                REQUIRE_FALSE(connected);
                REQUIRE_FALSE(second.tcpConnected());
                REQUIRE_FALSE(second.udpSynced());
                REQUIRE(first.tcpConnected());
                REQUIRE(host.readyIgCount() == 1);
                const auto peers = host.igSnapshot();
                REQUIRE(peers.size() == 1);
                REQUIRE(peers.front().channelId == 1);
            }
        }
    }
}

SCENARIO("relay forwards only the master IG TCP report to the platform",
         "[acceptance][bdd][platform][PLT-hello-ch]")
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

SCENARIO("relay does not forward a side-channel IG TCP report to the platform",
         "[acceptance][bdd][platform][PLT-hello-ch]")
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

SCENARIO("Host unpacks only the master IG TCP report when both IGs send",
         "[acceptance][bdd][platform][PLT-hello-ch]")
{
    GIVEN("a Host and two ready IGs on channel 0 and 1")
    {
        HostSync host;
        IgSync master;
        IgSync side;
        REQUIRE(startHostTwoIgs(host, master, side, 48600));

        std::vector<std::string> hostMsgs;
        host.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            hostMsgs.emplace_back(const_cast<CigiIGMsgV4&>(msg).GetMsg());
        });

        WHEN("both IGs send distinct TCP reports")
        {
            // 握手后仅 master TCP 入队；侧通道仍 recv，不入队。
            {
                auto& tcp = master.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3104);
                report.SetMsg("master");
                tcp << report;
                master.flushTcp();
            }
            {
                auto& tcp = side.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3105);
                report.SetMsg("side");
                tcp << report;
                side.flushTcp();
            }
            drainHostUntil(host, [&] { return !hostMsgs.empty(); });
            drainHostUntil(host, [&] { return hostMsgs.size() >= 2; });

            THEN("Host callbacks receive only the master payload")
            {
                REQUIRE(hostMsgs.size() == 1);
                REQUIRE(hostMsgs.front() == "master");
                REQUIRE(host.readyIgCount() == 2);
            }
        }
    }
}

SCENARIO("Host does not unpack a side-channel IG TCP report",
         "[acceptance][bdd][platform][PLT-hello-ch]")
{
    GIVEN("a Host and two ready IGs on channel 0 and 1")
    {
        HostSync host;
        IgSync master;
        IgSync side;
        REQUIRE(startHostTwoIgs(host, master, side, 48800));

        std::vector<std::string> hostMsgs;
        host.addCallback<CigiIGMsgV4>([&](const CigiIGMsgV4& msg) {
            hostMsgs.emplace_back(const_cast<CigiIGMsgV4&>(msg).GetMsg());
        });

        WHEN("only the side IG sends a TCP report")
        {
            // 反向用例：master 在线但不发；侧通道 TCP 不入队。
            {
                auto& tcp = side.outMsgWithSofTcp();
                CigiIGMsgV4 report;
                report.SetMsgID(0x3106);
                report.SetMsg("side-only");
                tcp << report;
                side.flushTcp();
            }
            drainHostUntil(host, [&] { return !hostMsgs.empty(); });

            THEN("Host callbacks receive no IG TCP report")
            {
                REQUIRE(hostMsgs.empty());
                REQUIRE(host.readyIgCount() == 2);
            }
        }
    }
}

SCENARIO("virtual IG does not join the platform before real IGs have gathered",
         "[acceptance][bdd][platform][PLT-ig-first]")
{
    GIVEN("a platform Host and a viewhost relay expecting two real IGs")
    {
        constexpr int kPlatform = 41000;
        constexpr int kViewhost = 41200;
        HostSync platform;
        REQUIRE(platform.initialize(makeTestHostConfig(kPlatform)));
        platform.run();

        HostDriver viewhost;
        REQUIRE(viewhost.initialize(makeRelayViewhostConfig(kViewhost, kPlatform, 2)));

        AND_GIVEN("only one real IG is ready")
        {
            IgSync realIg;
            const IgConfig realCfg = makeTestIgConfig(kViewhost + 1, kViewhost);
            REQUIRE(realIg.initialize(realCfg.udpPortRecv, 0));
            REQUIRE(realIg.connect(realCfg.target));
            REQUIRE(viewhost.readyIgCount() == 1);

            WHEN("the relay keeps ticking")
            {
                tickRelay(viewhost);

                THEN("the virtual IG has not connected and the platform has no ready IG")
                {
                    REQUIRE_FALSE(viewhost.virtualIgLinked());
                    REQUIRE(platform.readyIgCount() == 0);
                }
            }
        }
    }
}

SCENARIO("gathered real IGs let the virtual IG join the platform without becoming a viewhost peer",
         "[acceptance][bdd][platform][PLT-not-peer]")
{
    GIVEN("a platform Host and a viewhost relay expecting two real IGs")
    {
        constexpr int kPlatform = 41400;
        constexpr int kViewhost = 41600;
        HostSync platform;
        REQUIRE(platform.initialize(makeTestHostConfig(kPlatform)));
        platform.run();

        HostDriver viewhost;
        REQUIRE(viewhost.initialize(makeRelayViewhostConfig(kViewhost, kPlatform, 2)));

        AND_GIVEN("both real IGs are ready")
        {
            IgSync real0;
            IgSync real1;
            const IgConfig cfg0 = makeTestIgConfig(kViewhost + 1, kViewhost);
            const IgConfig cfg1 = makeTestIgConfig(kViewhost + 3, kViewhost);
            REQUIRE(real0.initialize(cfg0.udpPortRecv, 0));
            REQUIRE(real1.initialize(cfg1.udpPortRecv, 1));
            REQUIRE(real0.connect(cfg0.target));
            REQUIRE(real1.connect(cfg1.target));
            REQUIRE(viewhost.readyIgCount() == 2);

            WHEN("the relay ticks after gather")
            {
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
                while ((!viewhost.virtualIgLinked() || platform.readyIgCount() != 1) &&
                       std::chrono::steady_clock::now() < deadline)
                {
                    viewhost.pollRelay();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                THEN("the platform has one ready IG on channel 0, and viewhost snapshot lists only the real IGs")
                {
                    REQUIRE(viewhost.virtualIgLinked());
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

SCENARIO("relayed platform ownship is the IG eye, not the viewhost keyboard",
         "[acceptance][bdd][platform][PLT-filter-ownship]")
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

SCENARIO("HostDriver relay forwards platform UDP IGCtrl without adding another data-plane frame",
         "[acceptance][bdd][platform][PLT-filter-igctrl]")
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

SCENARIO("HostDriver relay forwards a platform TCP command with the original payload",
         "[acceptance][bdd][platform][PLT-tcp-pass]")
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

SCENARIO("HostDriver relay forwards an IG TCP report to the platform",
         "[acceptance][bdd][platform][PLT-tcp-return]")
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

SCENARIO("platform receives repeating collision-segment notifications through the relay",
         "[acceptance][bdd][platform][PLT-return-collision]")
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
                    hostSendFrame(platform, 0);
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

SCENARIO("relay keeps a side-channel collision notification off the platform",
         "[acceptance][bdd][platform][PLT-return-collision]")
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

SCENARIO("relay keeps real IG UDP SOF from both channels on viewhost",
         "[acceptance][bdd][platform][PLT-relay-sof]")
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

SCENARIO("HostDriver relay packSof after UDP forward is the only SOF the platform sees",
         "[acceptance][bdd][platform][PLT-relay-sof]")
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

SCENARIO("HostDriver relay packSof even when no real IG is ready",
         "[acceptance][bdd][platform][PLT-relay-sof]")
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
        REQUIRE(viewhost.virtualIgLinked());

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

SCENARIO("HostDriver viewhost TCP command does not break platform UDP frame numbers",
         "[acceptance][bdd][platform][PLT-splice-cmd]")
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

SCENARIO("relayed platform TCP command keeps the original payload",
         "[acceptance][bdd][platform][PLT-tcp-pass]")
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

SCENARIO("relay splits two Host TCP messages glued in one sendAll",
         "[acceptance][bdd][platform][PLT-frame-msg]")
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

SCENARIO("relay splits two IG TCP messages glued in one sendAll",
         "[acceptance][bdd][platform][PLT-frame-msg]")
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

SCENARIO("viewhost TCP weather is a separate message from the relayed platform command",
         "[acceptance][bdd][platform][PLT-splice-cmd]")
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

SCENARIO("viewhost TCP weather does not break relayed platform UDP frame numbers",
         "[acceptance][bdd][platform][PLT-splice-cmd]")
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

SCENARIO("relayed IG TCP report keeps the original payload",
         "[acceptance][bdd][platform][PLT-tcp-return]")
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

SCENARIO("relayed platform UDP IGCtrl keeps the original header",
         "[acceptance][bdd][platform][PLT-filter-igctrl]")
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

SCENARIO("virtual IG packSof after UDP forward is the only SOF the platform sees",
         "[acceptance][bdd][platform][PLT-relay-sof]")
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

SCENARIO("virtual IG packSof when no real IG is ready to receive UDP",
         "[acceptance][bdd][platform][PLT-relay-sof]")
{
    GIVEN("a platform Host linked to a virtual IG, and a viewhost Host with no ready IG")
    {
        HostSync platform;
        IgSync virtualIg;
        HostSync viewhost;
        REQUIRE(linkHostIg(platform, virtualIg, 45400));
        REQUIRE(viewhost.initialize(makeTestHostConfig(45600)));
        viewhost.run();

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

SCENARIO("relay forwards multiple platform TCP and UDP messages to the IG",
         "[acceptance][bdd][platform][PLT-multi-down]")
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

SCENARIO("relay forwards multiple IG TCP reports; IG UDP SOF stays on viewhost",
         "[acceptance][bdd][platform][PLT-multi-up]")
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

SCENARIO("viewhost refuses to stop forwarding before a platform ownship has arrived",
         "[acceptance][bdd][platform][PLT-relay-pause]")
{
    GIVEN("a linked relay that has only forwarded a platform IGCtrl without ownship")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 51200, 51400));
        retryUdpUntil(
            [&] {
                hostSendFrame(platform, 0);
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

SCENARIO("stopped forwarding blocks both directions and repeats the last platform eye",
         "[acceptance][bdd][platform][PLT-relay-pause]")
{
    GIVEN("a linked relay that has forwarded a platform ownship")
    {
        HostSync platform;
        HostDriver viewhost;
        IgSync realIg;
        const auto platformEye = makeEye(31.2, 121.5, 80.0, 45.0, 5.0);
        const auto laterEye = makeEye(40.0, 116.0, 200.0, 90.0, 0.0);
        REQUIRE(startHostDriverRelay(platform, viewhost, realIg, 51600, 51800));
        if (!seedPlatformOwnship(platform, viewhost, realIg, platformEye))
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

SCENARIO("resumed forwarding drops paused traffic and applies the next platform eye",
         "[acceptance][bdd][platform][PLT-relay-resume]")
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
        if (!seedPlatformOwnship(platform, viewhost, realIg, platformEye))
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

TEST_CASE("IG HELLO on TCP starts with CIGI SOF", "[unit][sync][wire-contract][HS-hello-cigi]")
{
    TcpSocket listener;
    REQUIRE(listener.listen(0));
    const int tcpPort = listener.localPort();
    REQUIRE(tcpPort > 0);

    IgSync ig;
    REQUIRE(ig.initialize(19117, 2));

    std::thread connecting([&] { ig.connect({"127.0.0.1", tcpPort, 19118}); });

    TcpSocket accepted;
    for (int i = 0; i < 100 && !listener.accept(accepted); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    REQUIRE(accepted.valid());

    unsigned char header[4]{};
    REQUIRE(accepted.recvAll(header, 4, 1000));
    accepted.close();
    listener.close();
    connecting.join();

    const std::uint16_t packetId = static_cast<std::uint16_t>(header[2] | (header[3] << 8));
    REQUIRE(packetId == CIGI_SOF_PACKET_ID_V4);
}

TEST_CASE("IgSync sendUdpMessage forwards raw datagram without adding SOF",
          "[unit][sync][CIGI-endpoint-udp]")
{
    HostSync host;
    IgSync ig;
    REQUIRE(linkHostIg(host, ig, 39800));

    host.drainIncoming();
    const auto sofBefore = host.sofReceivedCount();
    const auto sentBefore = ig.sofSentCount();

    std::vector<unsigned char> sof;
    REQUIRE(cigi_wire::packSof(42, sof));
    SyncInterface& sync = ig;
    // UDP 可丢：重发直到 Host 解到 SOF；合同不是单报必达。
    for (int i = 0; i < 10 && host.sofReceivedCount() == sofBefore; ++i)
    {
        sync.sendUdpMessage(sof);
        for (int j = 0; j < 8 && host.sofReceivedCount() == sofBefore; ++j)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            host.drainIncoming();
        }
    }

    REQUIRE(ig.sofSentCount() == sentBefore);
    if (host.sofReceivedCount() == sofBefore)
        SKIP("UDP datagram dropped");
    REQUIRE(host.sofReceivedCount() > sofBefore);
}

TEST_CASE("HostSync takeIncomingUdp consumes the UDP queue without drainIncoming",
          "[unit][sync][CIGI-endpoint-udp]")
{
    HostSync host;
    IgSync ig;
    REQUIRE(linkHostIg(host, ig, 39900));

    host.drainIncoming();
    const auto sofBefore = host.sofReceivedCount();

    SyncInterface& sync = host;
    std::vector<unsigned char> taken;
    for (int i = 0; i < 10 && taken.empty(); ++i)
    {
        ig.outMsgWithSofUdp();
        ig.flushUdp();
        const auto frames = collectFrames([&] { return sync.takeIncomingUdp(); }, 1);
        if (!frames.empty())
            taken = frames.back();
    }
    if (taken.empty())
        SKIP("UDP datagram dropped");
    REQUIRE(cigi_wire::isSofPacket(taken.data(), static_cast<int>(taken.size())));

    // 抽空可能迟到的报，避免随后 drainIncoming 把它们计成 SOF。
    for (int i = 0; i < 10; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        (void)sync.takeIncomingUdp();
    }

    host.drainIncoming();
    REQUIRE(host.sofReceivedCount() == sofBefore);
}

// =============================================================================
// 2. 帧节拍：CIGI IGCtrl / SOF / FreeRun（集成；握手为 CIGI HELLO + UDP_SYNC）
// 数据面线格式契约：IGCtrlV4 (+ 可选 EntityPositionCtrlV4) / SOFV4 —— 见 [wire-contract]。
// =============================================================================

SCENARIO("connected Host and IG enter RUNNING and exchange CIGI IGCtrl each update",
         "[integration][sync][status][cigi][CIGI-running-igctrl]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));
        REQUIRE(ig.connect(makeIgLocal().target));

        WHEN("Host runs and sends 10 CIGI IGCtrl frames while IG updates")
        {
            host.run();
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host, i * (1000.0 / 60.0));
                ig.drainIncoming();
                ig.update();
            }

            THEN("both are RUNNING and Host sent one IGCtrl per Update")
            {
                REQUIRE(host.status() == HostStatus::RUNNING);
                REQUIRE(ig.status() == IgStatus::RUNNING);
                REQUIRE(host.igCtrlSentCount() == kFrames);
                REQUIRE(approxAtMost(ig.igCtrlReceivedCount(), kFrames, 3));
            }
        }
    }
}

SCENARIO("Host records a matched IGCtrl-SOF RTT sample",
         "[integration][sync][meas][MEAS-sof-rtt]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));
        REQUIRE(ig.connect(makeIgLocal().target));

        WHEN("Host sends IGCtrl frames and IG replies SOF")
        {
            host.run();
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host, i * (1000.0 / 60.0));
                ig.drainIncoming(/*sendSof=*/true);
                ig.update();
            }

            THEN("the ready IG has a positive matched RTT sample")
            {
                host.drainIncoming();
                const auto peers = host.igSnapshot();
                REQUIRE_FALSE(peers.empty());
                const auto last = host.sofRttLast(peers.front().id);
                REQUIRE(last.has_value());
                REQUIRE(*last > std::chrono::microseconds{0});
                REQUIRE(peers.front().lastRtt == last);
                REQUIRE(peers.front().sofAge.has_value());
                REQUIRE(*peers.front().sofAge >= std::chrono::microseconds{0});
            }
        }
    }
}

SCENARIO("IG replies with one CIGI SOF per received IGCtrl", "[integration][sync][status][sof][cigi][CIGI-sof-echo]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));
        REQUIRE(ig.connect(makeIgLocal().target));

        WHEN("Host sends 10 CIGI IGCtrl and IG updates each frame (reply SOF)")
        {
            host.run();
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host, i * (1000.0 / 60.0));
                ig.drainIncoming(/*sendSof=*/true);
                ig.update();
            }

            THEN("SOF sent equals IGCtrl received; Host SOF count cannot exceed what IG sent")
            {
                host.drainIncoming();
                REQUIRE(ig.igCtrlReceivedCount() >= 1);
                REQUIRE(ig.sofSentCount() == ig.igCtrlReceivedCount());
                REQUIRE(host.sofReceivedCount() <= ig.sofSentCount());
            }
        }
    }
}

SCENARIO("Host keeps sending CIGI IGCtrl when IG never replies SOF",
         "[integration][sync][status][freerun][cigi][CIGI-freerun]")
{
    GIVEN("a connected Host and IG (send is never gated by SOF)")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));
        REQUIRE(ig.connect(makeIgLocal().target));

        WHEN("Host sends 10 CIGI IGCtrl while IG receives but never replies SOF")
        {
            host.run();
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host, i * (1000.0 / 60.0));
                ig.drainIncoming(/*sendSof=*/false);
                ig.update();
            }

            THEN("Host sent all IGCtrl without depending on SOF")
            {
                host.drainIncoming();
                REQUIRE(host.igCtrlSentCount() == kFrames);
                REQUIRE(host.sofReceivedCount() == 0);
                REQUIRE(ig.igCtrlReceivedCount() <= kFrames);
            }
        }
    }
}

SCENARIO("IG last received CIGI FrameCntr matches Host frame numbers",
         "[integration][sync][status][frame][cigi][CIGI-frame-cntr]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpPortRecv));
        REQUIRE(ig.connect(makeIgLocal().target));

        WHEN("Host sends CIGI IGCtrl FrameCntr 0..N-1 and IG updates each frame")
        {
            host.run();
            constexpr int kFrames = 10;
            std::uint32_t prevReceived = 0;
            int matchedFrames = 0;

            for (int i = 0; i < kFrames; ++i)
            {
                // Host 本轮 CIGI IGCtrl.FrameCntr == i（经 HostSync API 暴露为 lastIgCtrlFrameCntr）
                hostSendFrame(host, i * (1000.0 / 60.0));
                ig.drainIncoming();
                ig.update();

                if (ig.igCtrlReceivedCount() > prevReceived)
                {
                    // 异步 I/O 线程 + 队列下，一次 update 可能批量处理多条积压帧，
                    // lastIgCtrlFrameCntr 是最新帧号——断言其不超前于本帧已发送的帧号（方向不变）。
                    REQUIRE(ig.lastIgCtrlFrameCntr() <= static_cast<std::uint32_t>(i));
                    prevReceived = ig.igCtrlReceivedCount();
                    ++matchedFrames;
                }
            }

            THEN("at least one IGCtrl was received and FrameCntr values matched")
            {
                REQUIRE(matchedFrames >= 1);
                REQUIRE(ig.lastIgCtrlFrameCntr() < static_cast<std::uint32_t>(kFrames));
            }
        }
    }
}


// =============================================================================
// 独立 IG 配置 E2E：host 与 IG 双侧都走 sync 库独立配置文件
// （loadHostConfig / loadIgConfig → 各自 SynchronSystem），装配参数程序化注入。
// 验证外部 engine 脱离引擎整体配置使用 sync 的完整路径（sync模块化设计.md §4.1/§4.2）。
// =============================================================================

SCENARIO("host and IG both load standalone sync configs and exchange CIGI",
         "[acceptance][bdd][sync][standalone][cigi][CIGI-standalone]")
{
    GIVEN("host HostSync from hostConfig file, and IG SynchronSystem from igConfig file")
    {
        constexpr int kBase = 22000;

        // host 独立配置（udpRecv=base, tcp=base+100）。
        const TempConfigFile hostFile(
            std::string(R"({ "hostConfig": { "udpPortRecv": )") + std::to_string(kBase) +
            R"(, "tcpPort": )" + std::to_string(kBase + 100) + R"( } })");
        // IG 独立配置（本地 udpRecv=base+3，target 指向 host 的 tcp=base+100 / udpRecv=base）。
        const TempConfigFile igFile(
            std::string(R"({ "igConfig": { "udpPortRecv": )") + std::to_string(kBase + 3) +
            R"(, "targetAddr": "127.0.0.1", "targetTcpPort": )" + std::to_string(kBase + 100) +
            R"(, "targetUdpPortRecv": )" + std::to_string(kBase) + R"( } })");

        HostConfig host;
        std::string hostError;
        REQUIRE(loadHostConfig(hostFile.path(), host, &hostError));
        IgConfig ig;
        std::string igError;
        REQUIRE(loadIgConfig(igFile.path(), ig, &igError));

        // 两侧：host 用 HostSync 直发；IG 用 SynchronSystem（IG 决策器）。
        auto hostSync = std::make_unique<HostSync>();
        REQUIRE(hostSync->initialize(host));
        hostSync->run();

        auto igSync = SynchronSystem::create();
        // 装配参数程序化注入（外部 engine 不经 syncSystem 配置文件时的路径）。
        SyncSystemConfig igSystem;
        igSystem.channelId = 2;
        igSystem.offsetDeg = OffsetDeg{5.0, 0.0, 0.0};
        REQUIRE(igSync->initialize(ig, igSystem));

        WHEN("IG connects to host and both link")
        {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
            while (hostSync->readyIgCount() < 1 &&
                   std::chrono::steady_clock::now() < deadline)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            THEN("host sees the standalone IG and IG is linked")
            {
                REQUIRE(hostSync->readyIgCount() == 1);
                REQUIRE(igSync->igLinked());
            }

            THEN("CIGI IGCtrl and SOF flow both ways over TCP/UDP")
            {
                hostSync->drainIncoming();
                const std::uint32_t sentBefore = hostSync->igCtrlSentCount();
                const std::uint32_t recvBefore = igSync->igSync().igCtrlReceivedCount();
                const std::uint32_t sofBefore = hostSync->sofReceivedCount();

                constexpr int kTicks = 5;
                for (int i = 0; i < kTicks; ++i)
                {
                    hostSendFrame(*hostSync, i * 16.667); // 业务侧扇出 IGCtrl
                    igSync->preFrame();                   // IG 收 IGCtrl + 回 SOF
                }

                hostSync->drainIncoming();
                REQUIRE(hostSync->igCtrlSentCount() > sentBefore);
                REQUIRE(igSync->igSync().igCtrlReceivedCount() > recvBefore);
                REQUIRE(hostSync->sofReceivedCount() > sofBefore);
            }
        }
    }
}
