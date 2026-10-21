#include <aerovista/config/EntityCatalog.h>
#include <aerovista/sync/HostDataManager.h>

#include <utility>

namespace aerovista::sync
{
    namespace
    {
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
        std::vector<EntityAuthorityRow> rows;
        if (!config::loadHostEntityCatalog(path, rows, error))
            return false;

        _entities = std::move(rows);
        if (error)
            error->clear();
        return true;
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
