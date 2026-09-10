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

            // 眼点链路收敛（2026-08 / 2026-09）：UDP 链路通用捕获投递 ownship 原始报文，
            // 由业务侧回调完成眼点翻译 + offset 合成（Engine CameraDriver）——
            // 订阅注册在 Engine::registerIgCallbacks（转发到 CameraDriver::onOwnshipEyePose）。

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

        // 收包入口对等化（§8.1）：统一 drain TCP+UDP → 解包 → processor；帧级维护随后。
        // 眼点原始报文经 UDP 链路通用捕获多播投递，翻译 + offset 合成由业务侧
        // （engine 回调）完成（2026-09 眼点决策上移）；此处只收包 + IgSync 帧维护。
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
