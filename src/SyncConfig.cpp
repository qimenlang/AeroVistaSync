#include <aerovista/config/ConfigJson.h>
#include <aerovista/sync/SyncConfig.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace aerovista::sync
{
    HostConfig parseHostConfig(const nlohmann::json& obj)
    {
        config::rejectUnknownKeys(obj, {"udpPortSend", "udpPortRecv", "tcpPort"});
        HostConfig cfg;
        cfg.udpPortSend = config::requireInt(obj, "udpPortSend");
        cfg.udpPortRecv = config::requireInt(obj, "udpPortRecv");
        cfg.tcpPort = config::requireInt(obj, "tcpPort");
        return cfg;
    }

    IgConfig parseIgConfig(const nlohmann::json& obj)
    {
        config::rejectUnknownKeys(obj, {"udpPortSend", "udpPortRecv", "targetAddr", "targetTcpPort",
                                        "targetUdpPortRecv"});
        IgConfig cfg;
        cfg.udpPortSend = config::requireInt(obj, "udpPortSend");
        cfg.udpPortRecv = config::requireInt(obj, "udpPortRecv");
        cfg.targetAddr = config::requireString(obj, "targetAddr");
        cfg.targetTcpPort = config::requireInt(obj, "targetTcpPort");
        cfg.targetUdpPortRecv = config::requireInt(obj, "targetUdpPortRecv");
        return cfg;
    }

    namespace
    {
        /// 读文件 + 解析根对象；失败返回 false 并写 error。
        bool parseRootObject(const std::string& path, nlohmann::json& root, std::string* error)
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
                root = config::parseJsonText(oss.str());
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
        nlohmann::json root;
        if (!parseRootObject(path, root, error))
            return false;
        try
        {
            config::rejectUnknownKeys(root, {"hostConfig"});
            out = parseHostConfig(config::requireObjectValue(root, "hostConfig"));
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
        nlohmann::json root;
        if (!parseRootObject(path, root, error))
            return false;
        try
        {
            config::rejectUnknownKeys(root, {"igConfig"});
            out = parseIgConfig(config::requireObjectValue(root, "igConfig"));
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
