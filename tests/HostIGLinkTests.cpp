#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/CigiIncludes.h>
#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/IgSync.h>
#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SyncInterface.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"
#include "CigiIGMsgV4.h"
#include "CigiIGSession.h"
#include "CigiSOFV4.h"
#include "CigiSymbolTextDefV4.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Common.h"

using aerovista::sync::HostConfig;
using aerovista::sync::HostSync;
using aerovista::sync::IgConfig;
using aerovista::sync::IgSync;
using aerovista::sync::SyncInterface;
using aerovista::sync::TcpSocket;
namespace cigi_wire = aerovista::sync::cigi_wire;

// 直连 Host↔IG：握手、线格式、帧节拍。平台中继在 HostDriverTests.cpp。
// - 握手：CIGI HELLO（SOF+IGMsg，无 TCP ACK）+ UDP_SYNC / ACK —— §1 / PLT-hello-ch（直连）。
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
        return IgConfig{udpRecvPort, "127.0.0.1", 8100, 8000};
    }

    bool startHostTwoIgs(HostSync& host, IgSync& master, IgSync& side, int base)
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
    void connectUntil(const std::function<bool()>& linked)
    {
        for (int i = 0; i < 40 && !linked(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    void drainHostUntil(HostSync& host, const std::function<bool()>& done)
    {
        for (int i = 0; i < 40 && !done(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            host.drainIncoming();
        }
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

    /// IGCtrl 由 outMsgWithIgCtrlUdp() 自动前置（帧号/自计时时间戳）。
    // hostSendFrame 已抽到 SyncTestHelpers.h，这里不再重复定义。

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

    void hostSendEyePose(HostSync& host, const cigi_wire::EyePose& eye)
    {
        auto& omsg = host.outMsgWithIgCtrlUdp();
        cigi_wire::appendEye(omsg, &eye);
        host.flushUdp();
    }

    void pumpIgCtrlFrames(HostSync& host, IgSync& ig, int frames = 5)
    {
        for (int i = 0; i < frames; ++i)
        {
            hostSendFrame(host);
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

// =============================================================================
// Host 刚 initialize 时还没有 ready IG，核对 ready 台数为 0
// （多通道同步模块设计.md §11，码 HS-host-init-empty）。
// 只断言 readyIgCount()。initialize 成功与台数为 0 是两件事，都要过。
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
            const bool ok = ig.initialize(makeIgLocal().udpRecvPort);

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
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));

        WHEN("the IG connects to a Host endpoint")
        {
            const bool connected = ig.connect(makeIgLocal());

            THEN("connect fails and neither plane reports connected")
            {
                REQUIRE_FALSE(connected);
                REQUIRE_FALSE(ig.tcpConnected());
                REQUIRE_FALSE(ig.udpSynced());
            }
        }
    }
}

// =============================================================================
// Host 已在等待时 IG connect 成功，Host 看到 1 台 ready IG
// （多通道同步模块设计.md §11，码 HS-connect-ok）。
// 连接前台数为 0，成功后台数为 1。TCP 与 UDP 都要已同步。
// =============================================================================
SCENARIO("IG connects successfully when Host is already waiting", "[integration][sync][connect][HS-connect-ok]")
{
    GIVEN("a Host that has been initialized and is waiting for IGs")
    {
        HostSync host;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(host.readyIgCount() == 0);

        AND_GIVEN("an IG that has been initialized")
        {
            IgSync ig;
            REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));

            WHEN("the IG connects to the Host endpoint")
            {
                const bool connected = ig.connect(makeIgLocal());

                THEN("both planes are synced and Host has one ready IG")
                {
                    REQUIRE(connected);
                    REQUIRE(ig.tcpConnected());
                    REQUIRE(ig.udpSynced());
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
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
        REQUIRE(ig.connect(makeIgLocal()));
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

// =============================================================================
// TCP 端口正确、UDP 端口错误时 IG connect 失败，Host 仍然没有 ready IG
// （多通道同步模块设计.md §11，码 HS-connect-fail-udp-port）。
// 失败后 IG 两侧都未同步，Host 台数为 0。错的是 hostUdpRecvPort。
// =============================================================================
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
            REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));

            WHEN("the IG connects using a Host target with wrong UDP ports")
            {
                // Connect 使用 IgConfig::hostUdpRecvPort 作为 Host UDP 收端口。
                IgConfig badUdpConfig{8001, "127.0.0.1", 8100, 9999};
                const bool connected = ig.connect(badUdpConfig);

                THEN("overall connect fails and neither plane is ready")
                {
                    REQUIRE_FALSE(connected);
                    REQUIRE_FALSE(ig.tcpConnected());
                    REQUIRE_FALSE(ig.udpSynced());
                    REQUIRE(host.readyIgCount() == 0);
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
            REQUIRE(ig1.initialize(makeIgLocal(8001).udpRecvPort, 0));
            REQUIRE(ig2.initialize(makeIgLocal(8003).udpRecvPort, 1));

            REQUIRE(ig1.connect(makeIgLocal(8001)));
            REQUIRE(ig2.connect(makeIgLocal(8003)));

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
            REQUIRE(ig.initialize(makeIgLocal().udpRecvPort, 2));

            WHEN("the IG connects")
            {
                REQUIRE(ig.connect(makeIgLocal()));

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
        REQUIRE(first.initialize(makeIgLocal(8001).udpRecvPort, 1));
        REQUIRE(first.connect(makeIgLocal(8001)));
        REQUIRE(host.readyIgCount() == 1);

        WHEN("a second IG connects with the same channelId")
        {
            IgSync second;
            REQUIRE(second.initialize(makeIgLocal(8003).udpRecvPort, 1));
            const bool connected = second.connect(makeIgLocal(8003));

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

// =============================================================================
// 两台 IG 都经 TCP 上报时，Host 只解出 master 的 IGMsg
// （平台同步设计.md §6.3，码 PLT-hello-ch）。
// 侧通道完成 HELLO 后不再 recv。断言只看 Host 回调里没有侧通道正文，不验套接字缓冲。
// =============================================================================
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
            // 握手后仅 master 进入读循环；侧通道不再 recv。
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

TEST_CASE("IG HELLO on TCP starts with CIGI SOF", "[unit][sync][wire-contract][HS-hello-cigi]")
{
    TcpSocket listener;
    REQUIRE(listener.listen(0));
    const int tcpPort = listener.localPort();
    REQUIRE(tcpPort > 0);

    IgSync ig;
    REQUIRE(ig.initialize(19117, 2));

    std::thread connecting([&] { ig.connect(IgConfig{19117, "127.0.0.1", tcpPort, 19118}); });

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
// 握手完成后 Host 连发数据面 IGCtrl，IG 每帧收包，核对双方是否按帧交换
// （状态同步设计.md §10，码 CIGI-running-igctrl）。
// 不看状态枚举。到达条数允许少 3 条：UDP 可丢，不要求 10 条全到。
// =============================================================================
SCENARIO("connected Host and IG exchange one IGCtrl on each update",
         "[integration][sync][cigi][CIGI-running-igctrl]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
        REQUIRE(ig.connect(makeIgLocal()));

        WHEN("Host sends 10 IGCtrl frames while the IG drains and updates")
        {
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host);
                ig.drainIncoming();
                ig.update();
            }

            THEN("Host sent one IGCtrl per frame and the IG received them")
            {
                REQUIRE(host.igCtrlSentCount() == kFrames);
                REQUIRE(approxAtMost(ig.igCtrlReceivedCount(), kFrames, 3));
            }
        }
    }
}

// =============================================================================
// 握手完成后 Host 连发数据面 IGCtrl，IG 回 SOF，核对应用 RTT 为正
// （多通道同步验收测量设计.md §4.5，码 MEAS-rtt）。
// 按 channelId 取 rttLast，并与快照里的 lastRtt 一致。
// =============================================================================
SCENARIO("Host records a positive application RTT sample",
         "[integration][sync][meas][MEAS-rtt]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
        REQUIRE(ig.connect(makeIgLocal()));

        WHEN("Host sends IGCtrl frames and the IG echoes each one")
        {
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host);
                ig.drainIncoming(/*sendSof=*/true);
                ig.update();
            }

            THEN("the ready IG has a positive matched RTT sample")
            {
                host.drainIncoming();
                const auto peers = host.igSnapshot();
                REQUIRE_FALSE(peers.empty());
                const auto last = host.rttLast(peers.front().channelId);
                REQUIRE(last.has_value());
                REQUIRE(*last > std::chrono::microseconds{0});
                REQUIRE(peers.front().lastRtt == last);
                REQUIRE(peers.front().sofAge.has_value());
                REQUIRE(*peers.front().sofAge >= std::chrono::microseconds{0});
            }
        }
    }
}

// =============================================================================
// 握手完成后 Host 连发数据面 IGCtrl，IG 对每条收到的 IGCtrl 回一条 SOF，核对帧号回显
// （状态同步设计.md §6 / §10，码 CIGI-sof-echo）。
// =============================================================================

SCENARIO("IG replies with one CIGI SOF per received IGCtrl", "[integration][sync][status][sof][cigi][CIGI-sof-echo]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
        REQUIRE(ig.connect(makeIgLocal()));

        WHEN("Host sends 10 CIGI IGCtrl and IG updates each frame (reply SOF)")
        {
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host);
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

// =============================================================================
// 已连接的 Host 在 IG 不回 SOF 时仍发完数据面 IGCtrl，核对 FreeRun 按 SISO-STD-013-2014 §4.2.1 异步发帧
// （状态同步设计.md §3.1 / §10，码 CIGI-freerun）。
// §4.2.1：Host→IG 的间隔由 Host 自己决定，SOF 不决定下一包何时发。
// 同步 §4.2.2（IG 用 SOF 当心跳，Host 才回 IGCtrl）是尚未实现的 SofGated，本用例不覆盖。
// drainIncoming(sendSof=false) 故意不回 SOF。验收发送数等于发出帧数，且 Host 收到的 SOF 为 0；
// IG 收到数只要求不超过发出数，不要求必达。
// =============================================================================

SCENARIO("Host keeps sending CIGI IGCtrl when IG never replies SOF",
         "[integration][sync][status][freerun][cigi][CIGI-freerun]")
{
    GIVEN("a connected Host and IG (send is never gated by SOF)")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
        REQUIRE(ig.connect(makeIgLocal()));

        WHEN("Host sends 10 CIGI IGCtrl while IG receives but never replies SOF")
        {
            constexpr int kFrames = 10;
            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host);
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

// =============================================================================
// 握手完成后 Host 连发数据面 IGCtrl，核对 IG 解出的 FrameCntr 是否跟着 Host 帧号走
// （状态同步设计.md §10，码 CIGI-frame-cntr）。
// Host 数据面 `_dataFrameCounter` 从 0 起，每条 UDP IGCtrl 带一个帧号。不验收到达条数：
// 严格递增，且不超前于当时已发出的最大帧号；lastIgCtrlFrameCntr 等于最后一条。
// 一次 drainIncoming 可能消化多条积压，故不要求本轮 last == 本轮发送号。
// 一条都没解出来则无法核对，用例失败。
// =============================================================================

SCENARIO("IG received CIGI FrameCntr values follow Host frame numbers",
         "[integration][sync][status][frame][cigi][CIGI-frame-cntr]")
{
    GIVEN("a Host and an IG that have completed CIGI handshake")
    {
        HostSync host;
        IgSync ig;
        REQUIRE(host.initialize(makeHostLocal()));
        REQUIRE(ig.initialize(makeIgLocal().udpRecvPort));
        REQUIRE(ig.connect(makeIgLocal()));

        // 回调只记下解出的 FrameCntr，供 Then 核对。
        std::vector<std::uint32_t> frames;
        ig.addCallback<CigiIGCtrlV4>([&](const CigiIGCtrlV4& ctrl) { frames.push_back(ctrl.GetFrameCntr()); });

        WHEN("Host sends CIGI IGCtrl frames 0..N-1 and the IG processes each send")
        {
            constexpr int kFrames = 10;
            // 每条帧号解出时，Host 已经发出的最大帧号（0 起）。
            std::vector<std::uint32_t> hostFrameIDS;
            std::size_t seen = 0;

            for (int i = 0; i < kFrames; ++i)
            {
                hostSendFrame(host);
                ig.drainIncoming();
                ig.update();

                for (std::size_t j = seen; j < frames.size(); ++j)
                    hostFrameIDS.push_back(static_cast<std::uint32_t>(i));
                seen = frames.size();
            }

            THEN("received FrameCntr values increase, stay within frames already sent, and IG records the last one")
            {
                REQUIRE(!frames.empty());
                REQUIRE(framesIncreasing(frames));
                REQUIRE(hostFrameIDS.size() == frames.size());
                for (std::size_t j = 0; j < frames.size(); ++j)
                    REQUIRE(frames[j] <= hostFrameIDS[j]);
                REQUIRE(ig.igCtrlReceivedCount() == static_cast<std::uint32_t>(frames.size()));
                REQUIRE(ig.lastIgCtrlFrameCntr() == frames.back());
            }
        }
    }
}

// =============================================================================
// Host 与 IG 各自载入独立配置并连上后交换 CIGI，核对脱离引擎整体配置仍能走通 sync
// （sync模块化设计.md §4.1，码 CIGI-standalone）。
// Host：loadHostConfig → HostSync；IG：loadIgConfig → IgSync。
// channelId 程序化传入 IgSync::initialize（HELLO），不写进这两份传输配置。
// offsetDeg 由 Engine CameraDriver 消费，本用例不覆盖。
// 报文 Then 只要求发送数、IG 收到数、Host 的 SOF 数较交换前增加，不钉条数或帧号。
// =============================================================================

SCENARIO("host and IG both load standalone sync configs and exchange CIGI",
         "[acceptance][bdd][sync][standalone][cigi][CIGI-standalone]")
{
    GIVEN("host HostSync from hostConfig file, and IG IgSync from igConfig file")
    {
        constexpr int kBase = 22000;

        // host 独立配置（udpRecv=base, tcp=base+100）。
        const TempConfigFile hostFile(
            std::string(R"({ "hostConfig": { "udpPortRecv": )") + std::to_string(kBase) +
            R"(, "tcpPort": )" + std::to_string(kBase + 100) + R"( } })");
        // IG 独立配置（本地 udpRecv=base+3，target 指向 host 的 tcp=base+100 / udpRecv=base）。
        const TempConfigFile igFile(
            std::string(R"({ "igConfig": { "udpRecvPort": )") + std::to_string(kBase + 3) +
            R"(, "hostAddr": "127.0.0.1", "hostTcpPort": )" + std::to_string(kBase + 100) +
            R"(, "hostUdpRecvPort": )" + std::to_string(kBase) + R"( } })");

        HostConfig host;
        std::string hostError;
        REQUIRE(loadHostConfig(hostFile.path(), host, &hostError));
        IgConfig ig;
        std::string igError;
        REQUIRE(loadIgConfig(igFile.path(), ig, &igError));

        auto hostSync = std::make_unique<HostSync>();
        REQUIRE(hostSync->initialize(host));

        IgSync igSync;
        REQUIRE(igSync.initialize(ig.udpRecvPort, /*channelId=*/2));
        REQUIRE(igSync.connect(ig));

        WHEN("IG connects to host and both link")
        {
            connectUntil([&] { return hostSync->readyIgCount() >= 1 && igSync.linked(); });

            THEN("host sees the standalone IG and IG is linked")
            {
                REQUIRE(hostSync->readyIgCount() == 1);
                REQUIRE(igSync.linked());
            }

            THEN("CIGI IGCtrl and SOF flow both ways over TCP/UDP")
            {
                hostSync->drainIncoming();
                const std::uint32_t sentBefore = hostSync->igCtrlSentCount();
                const std::uint32_t recvBefore = igSync.igCtrlReceivedCount();
                const std::uint32_t sofBefore = hostSync->sofReceivedCount();

                constexpr int kTicks = 5;
                for (int i = 0; i < kTicks; ++i)
                {
                    hostSendFrame(*hostSync); // 业务侧扇出 IGCtrl
                    igSync.drainIncoming(true);
                    igSync.update();
                }

                hostSync->drainIncoming();
                REQUIRE(hostSync->igCtrlSentCount() > sentBefore);
                REQUIRE(igSync.igCtrlReceivedCount() > recvBefore);
                REQUIRE(hostSync->sofReceivedCount() > sofBefore);
            }
        }
    }
}
