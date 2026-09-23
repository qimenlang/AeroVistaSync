#include <aerovista/sync/HostDriver.h>

// 命令面/数据面测试报文（cigi梳理.md 链路矩阵；viewhost 报文自检）。
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
#include "CigiEntityPositionCtrlV4.h"
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

namespace aerovista::sync
{
    namespace
    {
        struct PacketProbe
        {
            const char* name;
            void (*send)(CigiOutgoingMsg&);
        };

#define PACKET_PROBE(PacketT, ...)                                                                                     \
    {                                                                                                                  \
        #PacketT, [](CigiOutgoingMsg& omsg)                                                                            \
        {                                                                                                              \
            PacketT packet;                                                                                            \
            __VA_ARGS__;                                                                                               \
            omsg << packet;                                                                                            \
        }                                                                                                              \
    }

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

    bool HostDriver::sendSymbolText(const std::string& text, std::string* error)
    {
        if (!_initialized)
        {
            if (error)
                *error = "Host 未初始化";
            return false;
        }
        if (text.empty())
        {
            if (error)
                *error = "空指令";
            return false;
        }

        CigiSymbolTextDefV4 packet(text.c_str());
        _host.outMsgWithIgCtrlTcp() << packet;
        _host.flushTcp();
        return true;
    }
} // namespace aerovista::sync
