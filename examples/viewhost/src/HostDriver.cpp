#include "HostDriver.h"

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"

namespace aerovista::viewhost
{
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
