#pragma once

#include <aerovista/sync/SyncConfig.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

using aerovista::sync::HostConfig;
using aerovista::sync::IgConfig;
using aerovista::sync::SyncSystemConfig;

class TempConfigFile
{
public:
    explicit TempConfigFile(const std::string& jsonBody)
    {
        _path = (std::filesystem::temp_directory_path() /
                 ("ave_engine_cfg_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json"))
                    .string();
        std::ofstream out(_path, std::ios::binary);
        out << jsonBody;
    }

    ~TempConfigFile()
    {
        std::error_code ec;
        std::filesystem::remove(_path, ec);
    }

    const std::string& path() const { return _path; }

private:
    std::string _path;
};

/// 写独立 entities.json + 指向它的 engine 通道配置（实体目录抽取后，测试装配用）。
/// 两个临时文件随对象析构清理；路径反斜杠转正斜杠嵌入 JSON，避免转义。
/// `extraFields` 为额外对象成员（不含前导逗号），例如 `"igConfig": { ... }`。
struct EntitiesConfig
{
    EntitiesConfig(const std::string& entitiesArrayBody, const std::string& cameraObject = {},
                   bool injectEllipsoidIfMissing = false, const std::string& extraFields = {})
    {
        entitiesFile = std::make_unique<TempConfigFile>(std::string(R"({ "entities": )") + entitiesArrayBody + " }");

        std::string cfg = "{";
        if (injectEllipsoidIfMissing)
            cfg += R"("injectEllipsoidIfMissing": true,)";
        std::string entitiesPath = entitiesFile->path();
        std::replace(entitiesPath.begin(), entitiesPath.end(), '\\', '/');
        cfg += R"("entitiesFilePath": ")" + entitiesPath + "\"";
        if (!cameraObject.empty())
            cfg += R"(, "camera": )" + cameraObject;
        if (!extraFields.empty())
            cfg += ", " + extraFields;
        cfg += R"(, "window": { "x": 0, "y": 0, "width": 640, "height": 480 } })";
        cfgFile = std::make_unique<TempConfigFile>(cfg);
    }

    std::unique_ptr<TempConfigFile> entitiesFile;
    std::unique_ptr<TempConfigFile> cfgFile;
};

// =============================================================================
// 测试共用：Host/IG 网络角色辅助（E2E 用）。
// 各测试文件调用时传不同 base 隔离端口，避免并行冲突。
// 命名 makeTest*，避免与 HostIGTests 握手期专用（无 base）的 makeHostLocal 冲突。
// =============================================================================

inline HostConfig makeTestHostConfig(int base)
{
    return HostConfig{base, base + 100};
}

inline IgConfig makeTestIgConfig(int udpRecvPort, int base)
{
    return IgConfig{udpRecvPort, {"127.0.0.1", base + 100, base}};
}

inline SyncSystemConfig makeTestSyncSystem(int channelId, bool requireConnectedIg = true)
{
    SyncSystemConfig syncSystem;
    syncSystem.channelId = channelId;
    syncSystem.requireConnectedIg = requireConnectedIg;
    return syncSystem;
}