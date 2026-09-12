#pragma once

#include <nlohmann/json.hpp>

#include <initializer_list>
#include <string>

/// 配置 JSON：语法解析用 nlohmann/json，schema 辅助仍在本命名空间。
/// 无 vsg / 引擎依赖（sync模块化设计.md §3.2 / §4.1）。
/// 供 loadHostConfig、loadIgConfig、HostDataManager 与引擎侧配置解析共用。
namespace aerovista::sync
{
    namespace sync_json
    {
        using JsonValue = nlohmann::json;
        using JsonObject = nlohmann::json;
        using JsonArray = nlohmann::json;

        /// 去掉 UTF-8 BOM 后按 RFC 8259 解析；语法错误抛 nlohmann::json::parse_error。
        JsonValue parseJsonText(std::string text);

        /// 在对象中查找键；不存在返回 nullptr。
        const JsonValue* find(const JsonObject& obj, const char* key);

        /// 拒绝某键的 JSON null 值（严格字段存在性）。
        void rejectNull(const JsonValue& v, const char* key);

        /// 拒绝任何不在 `allowed` 列表中的键（未知键策略，多通道同步模块设计.md §3.1）。
        void rejectUnknownKeys(const JsonObject& obj, std::initializer_list<const char*> allowed);

        /// 要求键存在且非 null。
        const JsonValue& requireValue(const JsonObject& obj, const char* key);

        /// 要求键存在且其值为对象。
        const JsonObject& requireObjectValue(const JsonObject& obj, const char* key);

        /// 要求当前值是对象（`key` 仅用于错误信息）。
        const JsonObject& requireObject(const JsonValue& v, const char* key);

        /// 要求数字（整数或浮点）。
        double requireNumber(const JsonObject& obj, const char* key);

        /// 要求整数（`1.0` 接受，`1.5` 拒绝）——两侧一致。
        int requireInt(const JsonObject& obj, const char* key);

        /// 要求字符串。
        std::string requireString(const JsonObject& obj, const char* key);

        /// 要求布尔。
        bool requireBool(const JsonObject& obj, const char* key);
    } // namespace sync_json
} // namespace aerovista::sync
