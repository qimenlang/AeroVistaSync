#include <aerovista/config/ConfigJson.h>
#include <aerovista/sync/SyncConfig.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace aerovista::sync
{
    HostConfig parseHostConfig(const nlohmann::json& obj)
    {
        config::rejectUnknownKeys(obj, {"udpPortRecv", "tcpPort"});
        HostConfig cfg;
        cfg.udpPortRecv = config::requireInt(obj, "udpPortRecv");
        cfg.tcpPort = config::requireInt(obj, "tcpPort");
        return cfg;
    }

    IgConfig parseIgConfig(const nlohmann::json& obj)
    {
        config::rejectUnknownKeys(obj, {"udpPortRecv", "targetAddr", "targetTcpPort", "targetUdpPortRecv"});
        IgConfig cfg;
        cfg.udpPortRecv = config::requireInt(obj, "udpPortRecv");
        cfg.target.addr = config::requireString(obj, "targetAddr");
        cfg.target.tcpPort = config::requireInt(obj, "targetTcpPort");
        cfg.target.udpPortRecv = config::requireInt(obj, "targetUdpPortRecv");
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

        /// `enable=true` 才填虚 IG；关闭时 JSON 里的 igConfig / expectedIgCount 忽略。
        void applyRelayBlock(const nlohmann::json& root, HostConfig& out)
        {
            const auto* relayValue = config::find(root, "relay");
            if (!relayValue)
                return;

            const auto& relay = config::requireObject(*relayValue, "relay");
            config::rejectUnknownKeys(relay, {"enable", "expectedIgCount"});
            if (config::find(relay, "enable"))
                out.relay.enable = config::requireBool(relay, "enable");
            if (!out.relay.enable)
                return;

            out.relay.expectedIgCount = config::requireInt(relay, "expectedIgCount");
            if (out.relay.expectedIgCount < 1)
                throw std::runtime_error("relay.expectedIgCount must be an integer >= 1");
            if (!config::find(root, "igConfig"))
                throw std::runtime_error("relay.enable requires igConfig");
            out.igConfig = parseIgConfig(config::requireObjectValue(root, "igConfig"));
            if (out.igConfig->udpPortRecv == out.udpPortRecv)
                throw std::runtime_error("igConfig.udpPortRecv must differ from hostConfig.udpPortRecv");
        }
    } // namespace

    bool loadHostConfig(const std::string& path, HostConfig& out, std::string* error)
    {
        nlohmann::json root;
        if (!parseRootObject(path, root, error))
            return false;
        try
        {
            config::rejectUnknownKeys(root, {"hostConfig", "relay", "igConfig"});
            out = parseHostConfig(config::requireObjectValue(root, "hostConfig"));
            applyRelayBlock(root, out);
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
