#include <aerovista/config/ConfigJson.h>
#include <aerovista/sync/HostDataManager.h>

#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace aerovista::sync
{
    namespace
    {
        using Json = nlohmann::json;
        using aerovista::config::find;
        using aerovista::config::parseJsonText;
        using aerovista::config::rejectNull;
        using aerovista::config::rejectUnknownKeys;
        using aerovista::config::requireInt;
        using aerovista::config::requireNumber;
        using aerovista::config::requireObject;
        using aerovista::config::requireObjectValue;
        using aerovista::config::requireString;
        using aerovista::config::requireValue;

        std::string basenameOfModel(const std::string& modelPath)
        {
            const auto slash = modelPath.find_last_of("/\\");
            return slash == std::string::npos ? modelPath : modelPath.substr(slash + 1);
        }

        std::string readTextFile(const std::string& path)
        {
            std::ifstream in(path);
            if (!in)
                throw std::runtime_error("failed to open entities file: " + path);

            std::ostringstream oss;
            oss << in.rdbuf();
            return oss.str();
        }

        std::array<double, 3> requireNumberTriple(const Json& obj, const char* key)
        {
            const Json& v = requireValue(obj, key);
            if (!v.is_array())
                throw std::runtime_error(std::string("missing/invalid array: ") + key);
            const Json& arr = v;
            if (arr.size() != 3)
                throw std::runtime_error(std::string("array length must be 3: ") + key);

            std::array<double, 3> out{};
            for (std::size_t i = 0; i < 3; ++i)
            {
                if (!arr[i].is_number())
                    throw std::runtime_error(std::string("array elements must be numbers: ") + key);
                out[i] = arr[i].get<double>();
            }
            return out;
        }

        EntityAuthorityPose parseEllipsoidPose(const Json& obj)
        {
            rejectUnknownKeys(obj, {"lla", "eulerYprDeg"});
            const Json& lla = requireObjectValue(obj, "lla");
            rejectUnknownKeys(lla, {"lat", "lon", "alt"});

            EntityAuthorityPose pose;
            pose.lat = requireNumber(lla, "lat");
            pose.lon = requireNumber(lla, "lon");
            pose.alt = requireNumber(lla, "alt");
            const std::array<double, 3> ypr = requireNumberTriple(obj, "eulerYprDeg");
            pose.yawDeg = ypr[0];
            pose.pitchDeg = ypr[1];
            pose.rollDeg = ypr[2];
            return pose;
        }

        void applyCatalogPose(const Json& entityObj, EntityAuthorityRow& row)
        {
            const Json* poseValue = find(entityObj, "pose");
            if (!poseValue)
                return;

            const Json& poseObj = requireObject(*poseValue, "pose");
            rejectUnknownKeys(poseObj, {"local", "ellipsoid"});
            if (const Json* local = find(poseObj, "local"))
                requireObject(*local, "local");
            if (const Json* ellipsoid = find(poseObj, "ellipsoid"))
                row.pose = parseEllipsoidPose(requireObject(*ellipsoid, "ellipsoid"));
        }

        EntityAuthorityState parseInitialState(const Json& obj)
        {
            const Json* v = find(obj, "initialEntityState");
            if (!v)
                return EntityAuthorityState::ACTIVE;
            rejectNull(*v, "initialEntityState");
            if (!v->is_string())
                throw std::runtime_error("missing/invalid string: initialEntityState");
            const std::string s = v->get<std::string>();
            if (s == "Active")
                return EntityAuthorityState::ACTIVE;
            if (s == "Standby")
                return EntityAuthorityState::STANDBY;
            throw std::runtime_error("invalid initialEntityState (only \"Active\" or \"Standby\"): " + s);
        }

        std::string parseOptionalName(const Json& obj, const std::string& fallback)
        {
            const Json* v = find(obj, "name");
            if (!v)
                return fallback;
            rejectNull(*v, "name");
            if (!v->is_string())
                throw std::runtime_error("missing/invalid string: name");
            return v->get<std::string>();
        }

        EntityAuthorityRow parseEntityRow(const Json& obj)
        {
            rejectUnknownKeys(obj, {"id", "name", "model", "initialEntityState", "pose"});

            const int id = requireInt(obj, "id");
            if (id < 1 || id > 65535)
                throw std::runtime_error("entity id out of range 1..65535");

            EntityAuthorityRow row;
            row.entityId = static_cast<std::uint16_t>(id);
            row.model = requireString(obj, "model");
            if (row.model.empty())
                throw std::runtime_error("entities[].model must be non-empty");
            row.name = parseOptionalName(obj, basenameOfModel(row.model));
            row.entityState = parseInitialState(obj);
            row.alpha = 255;
            applyCatalogPose(obj, row);
            return row;
        }

        std::vector<EntityAuthorityRow> parseEntitiesArray(const Json& value)
        {
            rejectNull(value, "entities");
            if (!value.is_array())
                throw std::runtime_error("entities must be an array");
            const Json& arr = value;
            if (arr.empty())
                throw std::runtime_error("entities must not be empty");

            std::vector<EntityAuthorityRow> rows;
            rows.reserve(arr.size());
            std::unordered_set<int> seenIds;
            for (const Json& item : arr)
            {
                const EntityAuthorityRow row = parseEntityRow(requireObject(item, "entities[]"));
                if (!seenIds.insert(row.entityId).second)
                    throw std::runtime_error("duplicate entity id");
                rows.push_back(row);
            }
            return rows;
        }

        std::vector<EntityAuthorityRow> parseCatalogFile(const std::string& path)
        {
            const Json rootValue = parseJsonText(readTextFile(path));
            if (!rootValue.is_object())
                throw std::runtime_error("entities file root must be a JSON object");

            const Json& root = rootValue;
            rejectUnknownKeys(root, {"entities"});
            const Json* entitiesValue = find(root, "entities");
            if (!entitiesValue)
                throw std::runtime_error("missing key: entities");
            return parseEntitiesArray(*entitiesValue);
        }

        EntityAuthorityRow* findEntity(std::vector<EntityAuthorityRow>& rows, std::uint16_t entityId)
        {
            for (EntityAuthorityRow& row : rows)
            {
                if (row.entityId == entityId)
                    return &row;
            }
            return nullptr;
        }

        const EntityAuthorityRow* findEntity(const std::vector<EntityAuthorityRow>& rows, std::uint16_t entityId)
        {
            for (const EntityAuthorityRow& row : rows)
            {
                if (row.entityId == entityId)
                    return &row;
            }
            return nullptr;
        }

        bool failUnknownEntity(std::string* error)
        {
            if (error)
                *error = "unknown entity id";
            return false;
        }

        bool succeed(std::string* error)
        {
            if (error)
                error->clear();
            return true;
        }

        CigiBaseEntityCtrl::EntityStateGrp toCigiEntityState(EntityAuthorityState state)
        {
            return state == EntityAuthorityState::ACTIVE ? CigiBaseEntityCtrl::Active : CigiBaseEntityCtrl::Standby;
        }
    } // namespace

    bool HostDataManager::loadEntityCatalog(const std::string& path, std::string* error)
    {
        try
        {
            std::vector<EntityAuthorityRow> rows = parseCatalogFile(path);
            _entities = std::move(rows);
            if (error)
                error->clear();
            return true;
        }
        catch (const std::exception& ex)
        {
            if (error)
                *error = ex.what();
            return false;
        }
    }

    std::vector<EntityAuthorityRow> HostDataManager::entitySnapshot() const
    {
        return _entities;
    }

    std::optional<EntityAuthorityRow> HostDataManager::entityRow(std::uint16_t entityId) const
    {
        const EntityAuthorityRow* row = findEntity(_entities, entityId);
        if (!row)
            return std::nullopt;
        return *row;
    }

    bool HostDataManager::setEntityState(std::uint16_t entityId, EntityAuthorityState state, std::string* error)
    {
        EntityAuthorityRow* row = findEntity(_entities, entityId);
        if (!row)
            return failUnknownEntity(error);
        row->entityState = state;
        return succeed(error);
    }

    bool HostDataManager::setEntityPose(std::uint16_t entityId, const EntityAuthorityPose& pose, std::string* error)
    {
        EntityAuthorityRow* row = findEntity(_entities, entityId);
        if (!row)
            return failUnknownEntity(error);
        row->pose = pose;
        return succeed(error);
    }

    bool HostDataManager::setEntityAlpha(std::uint16_t entityId, std::uint8_t alpha, std::string* error)
    {
        EntityAuthorityRow* row = findEntity(_entities, entityId);
        if (!row)
            return failUnknownEntity(error);
        row->alpha = alpha;
        return succeed(error);
    }

    std::optional<CigiEntityCtrlV4> HostDataManager::entityCtrlPacket(std::uint16_t entityId) const
    {
        const EntityAuthorityRow* row = findEntity(_entities, entityId);
        if (!row)
            return std::nullopt;

        CigiEntityCtrlV4 packet;
        packet.SetEntityID(row->entityId);
        packet.SetEntityState(toCigiEntityState(row->entityState));
        packet.SetAlpha(row->alpha);
        packet.SetEntityType(0);
        return packet;
    }

    std::optional<CigiEntityPositionCtrlV4> HostDataManager::entityPositionPacket(std::uint16_t entityId) const
    {
        const EntityAuthorityRow* row = findEntity(_entities, entityId);
        if (!row)
            return std::nullopt;

        CigiEntityPositionCtrlV4 packet;
        packet.SetEntityID(row->entityId);
        packet.SetAttachState(CigiBaseEntityPositionCtrl::Detach);
        packet.SetParentID(0);
        packet.SetLat(row->pose.lat);
        packet.SetLon(row->pose.lon);
        packet.SetAlt(row->pose.alt);
        packet.SetYaw(static_cast<float>(row->pose.yawDeg));
        packet.SetPitch(static_cast<float>(row->pose.pitchDeg));
        packet.SetRoll(static_cast<float>(row->pose.rollDeg));
        return packet;
    }
} // namespace aerovista::sync
