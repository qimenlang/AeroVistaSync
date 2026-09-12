#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SyncJson.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace aerovista::sync
{
    HostConfig parseHostConfig(const sync_json::JsonObject& obj)
    {
        sync_json::rejectUnknownKeys(obj, {"udpPortSend", "udpPortRecv", "tcpPort"});
        HostConfig cfg;
        cfg.udpPortSend = sync_json::requireInt(obj, "udpPortSend");
        cfg.udpPortRecv = sync_json::requireInt(obj, "udpPortRecv");
        cfg.tcpPort = sync_json::requireInt(obj, "tcpPort");
        return cfg;
    }

    IgConfig parseIgConfig(const sync_json::JsonObject& obj)
    {
        sync_json::rejectUnknownKeys(obj, {"udpPortSend", "udpPortRecv", "targetAddr", "targetTcpPort",
                                           "targetUdpPortRecv"});
        IgConfig cfg;
        cfg.udpPortSend = sync_json::requireInt(obj, "udpPortSend");
        cfg.udpPortRecv = sync_json::requireInt(obj, "udpPortRecv");
        cfg.targetAddr = sync_json::requireString(obj, "targetAddr");
        cfg.targetTcpPort = sync_json::requireInt(obj, "targetTcpPort");
        cfg.targetUdpPortRecv = sync_json::requireInt(obj, "targetUdpPortRecv");
        return cfg;
    }

    namespace
    {
        /// 读文件 + 解析根对象；失败返回 false 并写 error。
        bool parseRootObject(const std::string& path, sync_json::JsonObject& root, std::string* error)
        {
            try
            {
                std::ifstream in(path);
                if (!in)
                {
                    if (error)
                        *error = "failed to open config: " + path;
                    return false;
                }

                std::ostringstream oss;
                oss << in.rdbuf();
                root = sync_json::parseJsonText(oss.str());
                if (!root.is_object())
                    throw std::runtime_error("root must be a JSON object");
                return true;
            }
            catch (const std::exception& e)
            {
                if (error)
                    *error = e.what();
                return false;
            }
        }
    } // namespace

    bool loadHostConfig(const std::string& path, HostConfig& out, std::string* error)
    {
        sync_json::JsonObject root;
        if (!parseRootObject(path, root, error))
            return false;
        try
        {
            sync_json::rejectUnknownKeys(root, {"hostConfig"});
            out = parseHostConfig(sync_json::requireObjectValue(root, "hostConfig"));
            return true;
        }
        catch (const std::exception& e)
        {
            if (error)
                *error = e.what();
            return false;
        }
    }

    bool loadIgConfig(const std::string& path, IgConfig& out, std::string* error)
    {
        sync_json::JsonObject root;
        if (!parseRootObject(path, root, error))
            return false;
        try
        {
            sync_json::rejectUnknownKeys(root, {"igConfig"});
            out = parseIgConfig(sync_json::requireObjectValue(root, "igConfig"));
            return true;
        }
        catch (const std::exception& e)
        {
            if (error)
                *error = e.what();
            return false;
        }
    }
} // namespace aerovista::sync
