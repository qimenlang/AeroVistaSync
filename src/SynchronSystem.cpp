#include <aerovista/sync/SynchronSystem.h>

#include <iostream>

namespace aerovista::sync
{
    std::unique_ptr<SynchronSystem> SynchronSystem::create()
    {
        return std::make_unique<SynchronSystem>();
    }

    SynchronSystem::SynchronSystem() = default;

    SynchronSystem::~SynchronSystem()
    {
        shutdown();
    }

    bool SynchronSystem::initialize(const std::optional<IgConfig>& igConfig, const SyncSystemConfig& syncSystem)
    {
        shutdown();
        _channelId = syncSystem.channelId;

        if (igConfig.has_value())
        {
            _ig = std::make_unique<IgSync>();
            if (!_ig->initialize(*igConfig))
            {
                std::cerr << "SynchronSystem: IgSync initialize failed\n";
                shutdown();
                return false;
            }

            if (!_ig->connect(*igConfig))
            {
                if (syncSystem.requireConnectedIg)
                {
                    std::cerr << "SynchronSystem: IgSync connect failed\n";
                    shutdown();
                    return false;
                }
            }
        }

        return true;
    }

    void SynchronSystem::shutdown()
    {
        if (_ig)
        {
            _ig->shutdown();
            _ig.reset();
        }
    }

    void SynchronSystem::preFrame()
    {
        if (!_ig)
            return;

        // drain TCP+UDP → 解包 → 订阅回调（眼点合成在 Engine CameraDriver 回调内）；随后帧级维护。
        _ig->drainIncoming(/*sendSof=*/true);
        _ig->update();
    }

    IgSync& SynchronSystem::igSync()
    {
        return *_ig;
    }

    bool SynchronSystem::igLinked() const
    {
        return _ig && _ig->tcpConnected() && _ig->udpSynced();
    }
} // namespace aerovista::sync
