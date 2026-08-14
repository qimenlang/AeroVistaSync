#pragma once

#include <aerovista/sync/SyncJson.h>

#include <string>

namespace aerovista::sync
{
    /// 通道偏移：叠加在 Host 眼点之上（刚性阵列旋转，lla设计 §3.4）。
    struct OffsetDeg
    {
        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
    };

    /// 已连接但本帧无新 Host 眼点时的行为（多通道同步模块设计.md §4.4）。
    enum class HostEyeStalePolicy
    {
        REUSE_LAST,
        FREEZE
    };

    /// IG 侧配置 = 本地绑定 + 远端 Host 目标。
    struct IgConfig
    {
        std::string bindAddr;      ///< 本地绑定网卡
        int udpPortSend = 0;       ///< 本地 UDP 源端口
        int udpPortRecv = 0;       ///< 本地 UDP 接收端口
        std::string targetAddr;    ///< Host IP
        int targetTcpPort = 0;     ///< Host TCP 监听端口
        int targetUdpPortRecv = 0; ///< Host UDP 接收端口
    };

    /// Host 侧本地配置。
    struct HostConfig
    {
        std::string bindAddr; ///< 本地绑定网卡
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

    /// SynchronSystem 装配属性（sync模块化设计.md §4.2）。
    /// IG 侧消费为主（offset/stale/requireIgConnect），channelId 两端标识。
    struct SyncSystemConfig
    {
        int channelId = 0;
        OffsetDeg offsetDeg{};
        HostEyeStalePolicy hostEyeStalePolicy = HostEyeStalePolicy::REUSE_LAST;
        bool requireIgConnect = false;
    };

    struct SyncRoleConfig
    {
        bool enableHost = false;
        bool enableIg = false;
        HostConfig hostConfig{};
        IgConfig igConfig{};
    };

    /// 解析只含 `hostConfig` 块的文件（viewhost / 独立 Host 进程）。
    /// 顶层未知键拒绝。见 sync模块化设计.md §4.0。
    bool loadHostConfig(const std::string& path, HostConfig& out, std::string* error = nullptr);

    /// 解析只含 `igConfig` 块的文件（独立 IG 进程 / 外部引擎使用 sync 且不带引擎配置）。
    /// 顶层未知键拒绝。与 loadHostConfig 对称。见 sync模块化设计.md §4.1。
    bool loadIgConfig(const std::string& path, IgConfig& out, std::string* error = nullptr);

    /// 从已解析的 JSON 对象解析 `hostConfig` 块（与引擎侧共用）。
    HostConfig parseHostConfig(const sync_json::JsonObject& obj);

    /// 从已解析的 JSON 对象解析 `igConfig` 块（与引擎侧共用）。
    IgConfig parseIgConfig(const sync_json::JsonObject& obj);
} // namespace aerovista::sync
