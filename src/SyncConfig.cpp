#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SyncJson.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace aerovista::sync
{
    namespace sync_json
    {
        // =========================================================================
        // 校验 / 读取辅助（共享给引擎侧配置解析，见 sync模块化设计.md §3.2）
        // =========================================================================

        const JsonValue* find(const JsonObject& obj, const char* key)
        {
            const auto it = obj.find(key);
            return it == obj.end() ? nullptr : &it->second;
        }

        void rejectNull(const JsonValue& v, const char* key)
        {
            if (v.isNull())
                throw std::runtime_error(std::string("null is invalid: ") + key);
        }

        void rejectUnknownKeys(const JsonObject& obj, std::initializer_list<const char*> allowed)
        {
            for (const auto& [key, value] : obj)
            {
                (void)value;
                bool known = false;
                for (const char* a : allowed)
                {
                    if (key == a)
                    {
                        known = true;
                        break;
                    }
                }
                if (!known)
                    throw std::runtime_error("unknown key: " + key);
            }
        }

        const JsonValue& requireValue(const JsonObject& obj, const char* key)
        {
            const JsonValue* v = find(obj, key);
            if (!v)
                throw std::runtime_error(std::string("missing key: ") + key);
            rejectNull(*v, key);
            return *v;
        }

        const JsonObject& requireObjectValue(const JsonObject& obj, const char* key)
        {
            const JsonValue& v = requireValue(obj, key);
            if (!v.isObject())
                throw std::runtime_error(std::string("expected object: ") + key);
            return v.asObject();
        }

        const JsonObject& requireObjectValue(const JsonValue& v, const char* key)
        {
            rejectNull(v, key);
            if (!v.isObject())
                throw std::runtime_error(std::string("expected object: ") + key);
            return v.asObject();
        }

        double requireNumber(const JsonObject& obj, const char* key)
        {
            const JsonValue& v = requireValue(obj, key);
            if (!v.isNumber())
                throw std::runtime_error(std::string("expected number: ") + key);
            return v.asNumber();
        }

        int requireInt(const JsonObject& obj, const char* key)
        {
            const JsonValue& v = requireValue(obj, key);
            if (!v.isNumber())
                throw std::runtime_error(std::string("expected number: ") + key);
            const double n = v.asNumber();
            if (n != static_cast<double>(static_cast<long long>(n)))
                throw std::runtime_error(std::string("expected integer: ") + key);
            return static_cast<int>(n);
        }

        std::string requireString(const JsonObject& obj, const char* key)
        {
            const JsonValue& v = requireValue(obj, key);
            if (!v.isString())
                throw std::runtime_error(std::string("expected string: ") + key);
            return v.asString();
        }

        bool requireBool(const JsonObject& obj, const char* key)
        {
            const JsonValue& v = requireValue(obj, key);
            if (!v.isBool())
                throw std::runtime_error(std::string("expected bool: ") + key);
            return v.asBool();
        }
    } // namespace sync_json

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
        /// 读文件 + 去 BOM + 解析根对象；失败返回 false 并写 error。
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
                std::string text = oss.str();
                // Windows: 去掉开头的 UTF-8 BOM。
                if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
                    static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
                {
                    text.erase(0, 3);
                }

                sync_json::JsonParser parser(std::move(text));
                const sync_json::JsonValue rootValue = parser.parse();
                root = rootValue.asObject();
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
