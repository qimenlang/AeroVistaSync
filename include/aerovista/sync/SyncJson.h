#pragma once

#include <cctype>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

/// aerovistaSync 配置加载共用的最小 JSON 解析器。
/// 纯标准库；无 vsg / 引擎依赖（sync模块化设计.md §3.2）。
/// 供 loadHostConfig 与引擎侧配置解析使用。
namespace aerovista::sync
{
    namespace sync_json
    {
        struct JsonNull
        {
        };

        struct JsonValue;
        using JsonObject = std::unordered_map<std::string, JsonValue>;
        using JsonArray = std::vector<JsonValue>;

        struct JsonValue
        {
            using Storage = std::variant<JsonNull, bool, double, std::string, JsonArray, JsonObject>;
            Storage data;

            bool isNull() const { return std::holds_alternative<JsonNull>(data); }
            bool isObject() const { return std::holds_alternative<JsonObject>(data); }
            bool isArray() const { return std::holds_alternative<JsonArray>(data); }
            bool isString() const { return std::holds_alternative<std::string>(data); }
            bool isNumber() const { return std::holds_alternative<double>(data); }
            bool isBool() const { return std::holds_alternative<bool>(data); }

            const JsonObject& asObject() const { return std::get<JsonObject>(data); }
            const JsonArray& asArray() const { return std::get<JsonArray>(data); }
            const std::string& asString() const { return std::get<std::string>(data); }
            double asNumber() const { return std::get<double>(data); }
            bool asBool() const { return std::get<bool>(data); }
        };

        /// 递归下降 JSON 解析器（RFC-8259 子集，供引擎/sync 配置使用）。
        class JsonParser
        {
        public:
            explicit JsonParser(std::string text) :
                _text(std::move(text)) {}
            JsonValue parse()
            {
                skipWs();
                auto value = parseValue();
                skipWs();
                if (_pos != _text.size())
                    throw std::runtime_error("trailing data after JSON value");
                return value;
            }

        private:
            std::string _text;
            std::size_t _pos = 0;

            void skipWs()
            {
                while (_pos < _text.size() && std::isspace(static_cast<unsigned char>(_text[_pos])))
                    ++_pos;
            }

            char peek() const
            {
                if (_pos >= _text.size())
                    throw std::runtime_error("unexpected end of JSON");
                return _text[_pos];
            }

            char get()
            {
                const char c = peek();
                ++_pos;
                return c;
            }

            bool match(char expected)
            {
                skipWs();
                if (_pos < _text.size() && _text[_pos] == expected)
                {
                    ++_pos;
                    return true;
                }
                return false;
            }

            void expect(char expected)
            {
                skipWs();
                if (!match(expected))
                    throw std::runtime_error(std::string("expected '") + expected + "'");
            }

            JsonValue parseValue()
            {
                skipWs();
                const char c = peek();
                if (c == '{')
                    return JsonValue{parseObject()};
                if (c == '[')
                    return JsonValue{parseArray()};
                if (c == '"')
                    return JsonValue{parseString()};
                if (c == 't' || c == 'f')
                    return JsonValue{parseBool()};
                if (c == 'n')
                {
                    parseLiteral("null");
                    return JsonValue{JsonNull{}};
                }
                if (c == '-' || std::isdigit(static_cast<unsigned char>(c)))
                    return JsonValue{parseNumber()};
                throw std::runtime_error("invalid JSON value");
            }

            JsonObject parseObject()
            {
                expect('{');
                JsonObject obj;
                skipWs();
                if (match('}'))
                    return obj;

                while (true)
                {
                    skipWs();
                    const std::string key = parseString();
                    expect(':');
                    obj.emplace(key, parseValue());
                    skipWs();
                    if (match('}'))
                        break;
                    expect(',');
                }
                return obj;
            }

            JsonArray parseArray()
            {
                expect('[');
                JsonArray arr;
                skipWs();
                if (match(']'))
                    return arr;

                while (true)
                {
                    arr.push_back(parseValue());
                    skipWs();
                    if (match(']'))
                        break;
                    expect(',');
                }
                return arr;
            }

            std::string parseString()
            {
                expect('"');
                std::string out;
                while (true)
                {
                    if (_pos >= _text.size())
                        throw std::runtime_error("unterminated string");
                    const char c = get();
                    if (c == '"')
                        break;
                    if (c == '\\')
                    {
                        if (_pos >= _text.size())
                            throw std::runtime_error("unterminated escape");
                        const char e = get();
                        switch (e)
                        {
                        case '"':
                        case '\\':
                        case '/':
                            out.push_back(e);
                            break;
                        case 'b':
                            out.push_back('\b');
                            break;
                        case 'f':
                            out.push_back('\f');
                            break;
                        case 'n':
                            out.push_back('\n');
                            break;
                        case 'r':
                            out.push_back('\r');
                            break;
                        case 't':
                            out.push_back('\t');
                            break;
                        default:
                            throw std::runtime_error("unsupported escape");
                        }
                    }
                    else
                    {
                        out.push_back(c);
                    }
                }
                return out;
            }

            bool curIs(char c) const
            {
                return _pos < _text.size() && _text[_pos] == c;
            }

            bool curIsDigit() const
            {
                return _pos < _text.size() && std::isdigit(static_cast<unsigned char>(_text[_pos]));
            }

            void skipDigits()
            {
                while (curIsDigit())
                    ++_pos;
            }

            void parseFractionPart()
            {
                if (!curIs('.'))
                    return;
                ++_pos;
                skipDigits();
            }

            void parseExponentPart()
            {
                if (!curIs('e') && !curIs('E'))
                    return;
                ++_pos;
                if (curIs('+'))
                    ++_pos;
                else if (curIs('-'))
                    ++_pos;
                skipDigits();
            }

            double parseNumber()
            {
                bool negative = false;
                if (match('-'))
                    negative = true;

                const std::size_t intStart = _pos;
                skipDigits();
                parseFractionPart();
                parseExponentPart();

                const double value = std::stod(_text.substr(intStart, _pos - intStart));
                return negative ? -value : value;
            }

            bool parseBool()
            {
                if (match("true"))
                    return true;
                if (match("false"))
                    return false;
                throw std::runtime_error("invalid boolean");
            }

            void parseLiteral(const char* lit)
            {
                skipWs();
                for (const char* p = lit; *p; ++p)
                {
                    if (_pos >= _text.size() || _text[_pos] != *p)
                        throw std::runtime_error(std::string("invalid literal '") + lit + "'");
                    ++_pos;
                }
            }

            bool match(const char* lit)
            {
                skipWs();
                for (const char* p = lit; *p; ++p)
                {
                    if (_pos >= _text.size() || _text[_pos] != *p)
                        return false;
                    ++_pos;
                }
                return true;
            }
        };

        // =========================================================================
        // 校验 / 读取辅助（共享给 sync 配置解析与引擎侧配置解析）
        // =========================================================================

        /// 在对象中查找键；不存在返回 nullptr。
        const JsonValue* find(const JsonObject& obj, const char* key);

        /// 拒绝某键的 JSON null 值（严格字段存在性）。
        void rejectNull(const JsonValue& v, const char* key);

        /// 拒绝任何不在 `allowed` 列表中的键（未知键策略，§3.1）。
        void rejectUnknownKeys(const JsonObject& obj, std::initializer_list<const char*> allowed);

        /// 要求键存在且非 null。
        const JsonValue& requireValue(const JsonObject& obj, const char* key);

        /// 要求键的值为对象。
        const JsonObject& requireObjectValue(const JsonObject& obj, const char* key);

        /// 要求值为对象。
        const JsonObject& requireObjectValue(const JsonValue& v, const char* key);

        /// 要求数字。
        double requireNumber(const JsonObject& obj, const char* key);

        /// 要求整数（小数拒绝）——两侧一致。
        int requireInt(const JsonObject& obj, const char* key);

        /// 要求字符串。
        std::string requireString(const JsonObject& obj, const char* key);

        /// 要求布尔。
        bool requireBool(const JsonObject& obj, const char* key);
    } // namespace sync_json
} // namespace aerovista::sync
