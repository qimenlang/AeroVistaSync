#pragma once

#include <aerovista/sync/SyncJson.h>

#include <string>

namespace aerovista::sync
{
    /// Channel offset applied on top of the Host eye (rigid-array rotation, lla设计 §3.4).
    struct OffsetDeg
    {
        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
    };

    /// Behavior when linked but no new Host eye this frame (多通道同步模块设计.md §4.4).
    enum class HostEyeStalePolicy
    {
        REUSE_LAST,
        FREEZE
    };

    /// IG-side config = local bind + remote Host target (merged from former igLocal + hostEndpoint).
    struct IgConfig
    {
        std::string bindAddr;      ///< Local interface to bind (former igLocal.addr)
        int udpPortSend = 0;       ///< Local UDP source port (former igLocal.udpPortSend)
        int udpPortRecv = 0;       ///< Local UDP receive port (former igLocal.udpPortRecv)
        std::string targetAddr;    ///< Host IP (former hostEndpoint.addr)
        int targetTcpPort = 0;     ///< Host TCP listen port (former hostEndpoint.tcpPort)
        int targetUdpPortRecv = 0; ///< Host UDP receive port (former hostEndpoint.udpPortRecv)
    };

    /// Host-side local config (former hostLocal).
    struct HostConfig
    {
        std::string bindAddr; ///< Local interface to bind (former hostLocal.addr)
        int udpPortSend = 0;
        int udpPortRecv = 0;
        int tcpPort = 0;
    };

    enum class HostStatus
    {
        IDLE,
        RUNNING
    };

    enum class IgStatus
    {
        IDLE,
        RUNNING
    };

    enum class SendPace
    {
        FREE_RUN
    };

    enum class FrameGate
    {
        FREE_RUN,
        BARRIER
    };

    struct SyncPaceConfig
    {
        SendPace igCtrlSendPace = SendPace::FREE_RUN;
        FrameGate frameGate = FrameGate::FREE_RUN;
        double targetFps = 60.0;
        int barrierTimeoutMs = 8;
    };

    struct SyncRoleConfig
    {
        bool enableHost = false;
        bool enableIg = false;
        HostConfig hostConfig{};
        IgConfig igConfig{};
    };

    /// Parse a config file containing only the `hostConfig` block (viewhost / standalone
    /// Host process). Unknown top-level keys are rejected. See sync模块化设计.md §8.
    bool loadHostConfig(const std::string& path, HostConfig& out, std::string* error = nullptr);

    /// Parse a config file containing only the `igConfig` block (standalone IG process /
    /// external engine using sync without the engine config). Unknown top-level keys rejected.
    /// Symmetric to loadHostConfig. See sync模块化设计.md §8.1.
    bool loadIgConfig(const std::string& path, IgConfig& out, std::string* error = nullptr);

    /// Parse the `hostConfig` block from an already-parsed JSON object (shared with engine side).
    HostConfig parseHostConfig(const sync_json::JsonObject& obj);

    /// Parse the `igConfig` block from an already-parsed JSON object (shared with engine side).
    IgConfig parseIgConfig(const sync_json::JsonObject& obj);
} // namespace aerovista::sync
