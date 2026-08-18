#include "HostDriver.h"

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

    void HostDriver::update(double simTimeMs, const aerovista::sync::HostSync::EyePose* eye)
    {
        _host.update(simTimeMs, eye);
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
