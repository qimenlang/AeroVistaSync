#include "HostDriver.h"

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"

// 命令面/数据面测试报文（cigi梳理.md 链路矩阵；HostDriver 随机构造并发送）。
#include "CigiAccelerationCtrlV4.h"
#include "CigiAnimationCtrlV4.h"
#include "CigiArtPartCtrlV4.h"
#include "CigiAtmosCtrlV4.h"
#include "CigiCelestialCtrlV4.h"
#include "CigiCollDetSegDefV4.h"
#include "CigiCollDetVolDefV4.h"
#include "CigiCompCtrlV4.h"
#include "CigiConfClampEntityCtrlV4.h"
#include "CigiEarthModelDefV4.h"
#include "CigiEntityCtrlV4.h"
#include "CigiEnvCondReqV4.h"
#include "CigiEnvRgnCtrlV4.h"
#include "CigiHatHotReqV4.h"
#include "CigiLosSegReqV4.h"
#include "CigiLosVectReqV4.h"
#include "CigiMaritimeSurfaceCtrlV4.h"
#include "CigiMotionTrackCtrlV4.h"
#include "CigiPositionReqV4.h"
#include "CigiSensorCtrlV4.h"
#include "CigiShortArtPartCtrlV4.h"
#include "CigiShortCompCtrlV4.h"
#include "CigiShortSymbolCtrlV4.h"
#include "CigiSymbolCircleDefV4.h"
#include "CigiSymbolCloneV4.h"
#include "CigiSymbolCtrlV4.h"
#include "CigiSymbolPolygonDefV4.h"
#include "CigiSymbolSurfaceDefV4.h"
#include "CigiSymbolTextDefV4.h"
#include "CigiSymbolTexturedCircleDefV4.h"
#include "CigiSymbolTexturedPolygonDefV4.h"
#include "CigiTerrestrialSurfaceCtrlV4.h"
#include "CigiVelocityCtrlV4.h"
#include "CigiViewCtrlV4.h"
#include "CigiViewDefV4.h"
#include "CigiWaveCtrlV4.h"
#include "CigiWeatherCtrlV4.h"

#include <cstddef>
#include <random>
#include <string>

namespace aerovista::viewhost
{
    namespace
    {
        /// 一条测试报文：类名（HUD/MFC 对照显示）+ 构造并塞入出站消息的发送器。
        struct PacketProbe
        {
            const char* name;
            void (*send)(CigiOutgoingMsg&);
        };

        /// 生成报文探测项：默认构造 `PacketT`，可选宏变参做字段填充（如 EntityPositionCtrl 设 EntityID）。
#define PACKET_PROBE(PacketT, ...)                                                                                     \
    {                                                                                                                  \
        #PacketT, [](CigiOutgoingMsg& omsg)                                                                           \
        {                                                                                                              \
            PacketT packet;                                                                                            \
            __VA_ARGS__;                                                                                               \
            omsg << packet;                                                                                            \
        }                                                                                                              \
    }

        /// 命令面（TCP）：Host→IG 一次性/配置/请求/符号类。
        const PacketProbe kTcpProbes[] = {
            PACKET_PROBE(CigiEntityCtrlV4),
            PACKET_PROBE(CigiEntityPositionCtrlV4, packet.SetEntityID(7)),
            PACKET_PROBE(CigiArtPartCtrlV4),
            PACKET_PROBE(CigiShortArtPartCtrlV4),
            PACKET_PROBE(CigiCompCtrlV4),
            PACKET_PROBE(CigiShortCompCtrlV4),
            PACKET_PROBE(CigiAnimationCtrlV4),
            PACKET_PROBE(CigiViewDefV4),
            PACKET_PROBE(CigiSensorCtrlV4),
            PACKET_PROBE(CigiMotionTrackCtrlV4),
            PACKET_PROBE(CigiAtmosCtrlV4),
            PACKET_PROBE(CigiCelestialCtrlV4),
            PACKET_PROBE(CigiEnvRgnCtrlV4),
            PACKET_PROBE(CigiWeatherCtrlV4),
            PACKET_PROBE(CigiMaritimeSurfaceCtrlV4),
            PACKET_PROBE(CigiTerrestrialSurfaceCtrlV4),
            PACKET_PROBE(CigiWaveCtrlV4),
            PACKET_PROBE(CigiEarthModelDefV4),
            PACKET_PROBE(CigiCollDetSegDefV4),
            PACKET_PROBE(CigiCollDetVolDefV4),
            PACKET_PROBE(CigiHatHotReqV4),
            PACKET_PROBE(CigiLosSegReqV4),
            PACKET_PROBE(CigiLosVectReqV4),
            PACKET_PROBE(CigiPositionReqV4),
            PACKET_PROBE(CigiEnvCondReqV4),
            PACKET_PROBE(CigiSymbolCtrlV4),
            PACKET_PROBE(CigiShortSymbolCtrlV4),
            PACKET_PROBE(CigiSymbolSurfaceDefV4),
            PACKET_PROBE(CigiSymbolTextDefV4),
            PACKET_PROBE(CigiSymbolCircleDefV4),
            PACKET_PROBE(CigiSymbolPolygonDefV4),
            PACKET_PROBE(CigiSymbolTexturedCircleDefV4),
            PACKET_PROBE(CigiSymbolTexturedPolygonDefV4),
            PACKET_PROBE(CigiSymbolCloneV4),
        };

        /// 数据面（UDP）：Host→IG 持续/每帧控制类（IGCtrl 与 ownship 眼点由 update 常态化发送，不在此列）。
        const PacketProbe kUdpProbes[] = {
            PACKET_PROBE(CigiConfClampEntityCtrlV4),
            PACKET_PROBE(CigiVelocityCtrlV4),
            PACKET_PROBE(CigiAccelerationCtrlV4),
            PACKET_PROBE(CigiViewCtrlV4),
        };

#undef PACKET_PROBE

        template <std::size_t probeCount>
        const PacketProbe& pickRandomProbe(const PacketProbe (&probes)[probeCount])
        {
            static std::mt19937 rng{std::random_device{}()};
            std::uniform_int_distribution<std::size_t> dist(0, probeCount - 1);
            return probes[dist(rng)];
        }
    } // namespace

    HostDriver::~HostDriver()
    {
        shutdown();
    }

    bool HostDriver::initialize(const aerovista::sync::HostConfig& config, std::string* error)
    {
        if (!_host.initialize(config))
        {
            if (error)
                *error = "HostSync::initialize failed";
            return false;
        }
        _host.run();
        _initialized = true;
        return true;
    }

    void HostDriver::shutdown()
    {
        if (_initialized)
        {
            _host.shutdown();
            _initialized = false;
        }
    }

    void HostDriver::update(const aerovista::sync::cigi_wire::EyePose* eye)
    {
        auto& omsg = _host.outMsgWithIgCtrlUdp();
        aerovista::sync::cigi_wire::appendEye(omsg, eye);
        _host.flushUdp();
    }

    void HostDriver::sendEntityPose(std::uint16_t entityId, double lat, double lon, double alt, double yawDeg,
                                    double pitchDeg, double rollDeg)
    {
        // 命令面（TCP）一次性摆放：CCL 要求 Host 消息以 IGCtrl 开头（outMsgWithIgCtrlTcp 自动前置）。
        auto& tcp = _host.outMsgWithIgCtrlTcp();
        CigiEntityPositionCtrlV4 pose;
        pose.SetEntityID(entityId);
        pose.SetAttachState(CigiBaseEntityPositionCtrl::Detach); // 绝对 LLA
        pose.SetLat(lat, false);
        pose.SetLon(lon, false);
        pose.SetAlt(alt, false);
        pose.SetYaw(static_cast<float>(yawDeg), false);
        pose.SetPitch(static_cast<float>(pitchDeg), false);
        pose.SetRoll(static_cast<float>(rollDeg), false);
        tcp << pose;
        _host.flushTcp();
    }

    std::string HostDriver::sendRandomTcpPacket()
    {
        const PacketProbe& probe = pickRandomProbe(kTcpProbes);
        auto& omsg = _host.outMsgWithIgCtrlTcp();
        probe.send(omsg);
        _host.flushTcp();
        return probe.name;
    }

    std::string HostDriver::sendRandomUdpPacket()
    {
        const PacketProbe& probe = pickRandomProbe(kUdpProbes);
        auto& omsg = _host.outMsgWithIgCtrlUdp();
        probe.send(omsg);
        _host.flushUdp();
        return probe.name;
    }

    void HostDriver::pollIncoming()
    {
        _host.drainIncoming();
    }

    bool HostDriver::isRunning() const
    {
        return _host.status() == aerovista::sync::HostStatus::RUNNING;
    }

    int HostDriver::readyIgCount() const
    {
        return _host.readyIgCount();
    }

    std::uint32_t HostDriver::igCtrlSentCount() const
    {
        return _host.igCtrlSentCount();
    }

    std::uint32_t HostDriver::sofReceivedCount() const
    {
        return _host.sofReceivedCount();
    }
} // namespace aerovista::viewhost
