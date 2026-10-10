#include <aerovista/sync/HostDriver.h>
#include <aerovista/sync/IgSync.h>

#include "CigiEntityCtrlV4.h"
#include "CigiEntityPositionCtrlV4.h"

#include <memory>
#include <optional>
#include <string>

namespace aerovista::sync
{
    namespace
    {
        bool failUnknownEntity(std::string* error)
        {
            if (error)
                *error = "unknown entity id";
            return false;
        }

        bool appendEntityCtrl(CigiOutgoingMsg& tcp, const HostDataManager& data, std::uint16_t entityId,
                              std::string* error)
        {
            std::optional<CigiEntityCtrlV4> packet = data.entityCtrlPacket(entityId);
            if (!packet)
                return failUnknownEntity(error);
            tcp << *packet;
            return true;
        }

        bool appendEntityPosition(CigiOutgoingMsg& tcp, const HostDataManager& data, std::uint16_t entityId,
                                  std::string* error)
        {
            std::optional<CigiEntityPositionCtrlV4> packet = data.entityPositionPacket(entityId);
            if (!packet)
                return failUnknownEntity(error);
            tcp << *packet;
            return true;
        }
    } // namespace

    HostDriver::HostDriver() = default;

    HostDriver::~HostDriver()
    {
        shutdown();
    }

    bool HostDriver::initialize(const HostConfig& config, std::string* error)
    {
        if (!_host.initialize(config))
        {
            if (error)
                *error = "HostSync::initialize failed";
            return false;
        }
        _host.run();
        _relay = config.relay;
        _igConfig = config.igConfig;
        _cachedOwnship.reset();
        _relayForwarding = true;
        _initialized = true;
        return true;
    }

    void HostDriver::shutdown()
    {
        if (_initialized)
        {
            _virtualIg.reset();
            _host.shutdown();
            _cachedOwnship.reset();
            _relayForwarding = true;
            _initialized = false;
        }
    }

    void HostDriver::update(const cigi_wire::EyePose* eye)
    {
        if (_relay.enable && _relayForwarding)
            return;
        auto& omsg = _host.outMsgWithIgCtrlUdp();
        cigi_wire::appendEye(omsg, eye);
        _host.flushUdp();
    }

    bool HostDriver::loadEntityCatalog(const std::string& path, std::string* error)
    {
        return _data.loadEntityCatalog(path, error);
    }

    std::vector<EntityAuthorityRow> HostDriver::entitySnapshot() const
    {
        return _data.entitySnapshot();
    }

    std::optional<EntityAuthorityRow> HostDriver::entityRow(std::uint16_t entityId) const
    {
        return _data.entityRow(entityId);
    }

    bool HostDriver::setEntityCtrl(std::uint16_t entityId, EntityAuthorityState state, std::uint8_t alpha,
                                   std::string* error)
    {
        if (!_data.setEntityState(entityId, state, error))
            return false;
        return _data.setEntityAlpha(entityId, alpha, error);
    }

    bool HostDriver::setEntityPose(std::uint16_t entityId, const EntityAuthorityPose& pose, std::string* error)
    {
        return _data.setEntityPose(entityId, pose, error);
    }

    bool HostDriver::sendEntity(std::uint16_t entityId, EntitySend send, std::string* error)
    {
        if (!send.entityCtrl && !send.entityPosition)
            return true;

        auto& tcp = _host.outMsgWithIgCtrlTcp();
        if (send.entityCtrl && !appendEntityCtrl(tcp, _data, entityId, error))
            return false;
        if (send.entityPosition && !appendEntityPosition(tcp, _data, entityId, error))
            return false;
        _host.flushTcp();
        return true;
    }

    void HostDriver::pollIncoming()
    {
        _host.drainIncoming();
    }

    void HostDriver::pollRelay()
    {
        if (!_relay.enable)
            return;
        if (shouldConnectVirtualIg())
            connectVirtualIgToPlatform();
        if (!virtualIgHandshakeDone())
            return;
        if (!_relayForwarding)
        {
            dropQueuedRelay();
            return;
        }
        forwardFromPlatform();
        forwardToPlatform();
    }

    void HostDriver::forwardFromPlatform()
    {
        for (const auto& msg : _virtualIg->takeIncomingTcp())
            _host.sendTcpMessage(msg);
        for (const auto& dgram : _virtualIg->takeIncomingUdp())
        {
            const bool delivered = _host.readyIgCount() > 0;
            _host.sendUdpMessage(dgram);
            if (delivered)
                rememberForwardedOwnship(dgram);
            _virtualIg->sendSofForIgCtrl(dgram);
        }
    }

    void HostDriver::forwardToPlatform()
    {
        for (const auto& msg : _host.takeIncomingTcp())
            _virtualIg->sendTcpMessage(msg);
        for (const auto& body : _host.takeMasterUdpRelayBodies())
            _virtualIg->sendUdpAfterSof(body);
    }

    bool HostDriver::setRelayForwarding(bool forwarding)
    {
        if (!_relay.enable)
            return false;
        // 如果没有缓存眼点数据，则拒绝关闭转发
        if (!forwarding && !_cachedOwnship)
            return false;

        dropQueuedRelay();
        _relayForwarding = forwarding;
        return true;
    }

    bool HostDriver::relayForwarding() const
    {
        return _relayForwarding;
    }

    std::optional<cigi_wire::EyePose> HostDriver::relayEye() const
    {
        if (_relayForwarding)
            return std::nullopt;
        return _cachedOwnship;
    }

    void HostDriver::dropQueuedRelay()
    {
        if (_virtualIg)
        {
            _virtualIg->takeIncomingTcp();
            _virtualIg->takeIncomingUdp();
        }
        _host.takeIncomingTcp();
        _host.takeMasterUdpRelayBodies();
    }

    void HostDriver::rememberForwardedOwnship(const std::vector<unsigned char>& dgram)
    {
        if (auto eye = cigi_wire::ownshipFromMessage(dgram.data(), static_cast<int>(dgram.size())))
            _cachedOwnship = *eye;
    }

    bool HostDriver::virtualIgHandshakeDone() const
    {
        return _virtualIg && _virtualIg->tcpConnected() && _virtualIg->udpSynced();
    }

    bool HostDriver::shouldConnectVirtualIg() const
    {
        if (!_initialized || !_relay.enable || !_igConfig)
            return false;
        if (virtualIgHandshakeDone())
            return false;
        return _host.readyIgCount() >= _relay.expectedIgCount;
    }

    void HostDriver::connectVirtualIgToPlatform()
    {
        if (!_virtualIg)
            _virtualIg = std::make_unique<IgSync>();
        // 虚 IG 对平台 HELLO 恒 channelId=0（平台同步设计.md §6.3）。
        if (!_virtualIg->initialize(_igConfig->udpPortRecv, 0))
            return;
        _virtualIg->connect(_igConfig->target);
    }

    bool HostDriver::isRunning() const
    {
        return _host.status() == HostStatus::RUNNING;
    }

    int HostDriver::readyIgCount() const
    {
        return _host.readyIgCount();
    }

    std::vector<IgConnection> HostDriver::igSnapshot() const
    {
        return _host.igSnapshot();
    }

    std::uint32_t HostDriver::igCtrlSentCount() const
    {
        return _host.igCtrlSentCount();
    }

    std::uint32_t HostDriver::sofReceivedCount() const
    {
        return _host.sofReceivedCount();
    }
} // namespace aerovista::sync
