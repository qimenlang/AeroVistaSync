#pragma once

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <optional>
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

    /// IG 连 Host 的远端目标（TCP connect + UDP 发送）。
    /// JSON 仍扁平：`targetAddr` / `targetTcpPort` / `targetUdpPortRecv` → 本结构。
    struct HostTarget
    {
        std::string addr;    ///< Host IP
        int tcpPort = 0;     ///< Host TCP 监听端口
        int udpPortRecv = 0; ///< Host UDP 接收端口
    };

    /// IG 侧配置 = 本地 UDP 接收端口 + 远端 Host 目标。
    struct IgConfig
    {
        int udpPortRecv = 0; ///< 本地 UDP 接收端口
        HostTarget target;   ///< 远端 Host
    };

    /// viewhost 中继开关（平台同步设计.md §9）。仅 `enable=true` 时消费 `HostConfig::igConfig`。
    struct RelayConfig
    {
        bool enable = false;
        int expectedIgCount = 0;
    };

    /// Host 侧本地配置。
    struct HostConfig
    {
        int udpPortRecv = 0;
        int tcpPort = 0;
        RelayConfig relay{};
        /// 虚 IG；`relay.enable=false` 时为空（JSON 有也忽略）。
        std::optional<IgConfig> igConfig;
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

    /// SynchronSystem 装配属性（sync模块化设计.md §4.2）。
    /// `channelId` / `requireConnectedIg` 由 SynchronSystem 消费；`offsetDeg` 由 Engine CameraDriver 消费。
    struct SyncSystemConfig
    {
        int channelId = 0;
        OffsetDeg offsetDeg{};
        bool requireConnectedIg = false;
    };

    /// 解析 Host 进程配置（viewhost）。本地调试只含 `hostConfig`；
    /// 中继时可含 `relay` / `igConfig`（平台同步设计.md §9）。顶层未知键拒绝。
    bool loadHostConfig(const std::string& path, HostConfig& out, std::string* error = nullptr);

    /// 解析只含 `igConfig` 块的文件（独立 IG 进程 / 外部引擎使用 sync 且不带引擎配置）。
    /// 顶层未知键拒绝。与 loadHostConfig 对称。见 sync模块化设计.md §4.1。
    bool loadIgConfig(const std::string& path, IgConfig& out, std::string* error = nullptr);

    /// 从已解析的 JSON 对象解析 `hostConfig` 块（与引擎侧共用）。
    HostConfig parseHostConfig(const nlohmann::json& obj);

    /// 从已解析的 JSON 对象解析 `igConfig` 块（与引擎侧共用）。
    IgConfig parseIgConfig(const nlohmann::json& obj);
} // namespace aerovista::sync
