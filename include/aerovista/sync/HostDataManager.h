#pragma once

#include <aerovista/config/EntityCatalog.h>
#include <aerovista/sync/CigiIncludes.h>

#include "CigiEntityCtrlV4.h"
#include "CigiEntityPositionCtrlV4.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace aerovista::sync
{
    using config::EntityAuthorityPose;
    using config::EntityAuthorityRow;
    using config::EntityAuthorityState;

    /// Host 侧权威表门面（sync模块化设计.md §3.4）。首版只维护实体表。
    /// 不持 socket、不 flush。
    class HostDataManager
    {
    public:
        /// 调 `loadHostEntityCatalog` 建表。成功则替换整张实体表；失败不改已有表。
        bool loadEntityCatalog(const std::string& path, std::string* error = nullptr);

        std::vector<EntityAuthorityRow> entitySnapshot() const;
        std::optional<EntityAuthorityRow> entityRow(std::uint16_t entityId) const;

        /// 运行期更新一行；未知 id 失败且不改表。
        bool setEntityState(std::uint16_t entityId, EntityAuthorityState state, std::string* error = nullptr);
        bool setEntityPose(std::uint16_t entityId, const EntityAuthorityPose& pose, std::string* error = nullptr);
        bool setEntityAlpha(std::uint16_t entityId, std::uint8_t alpha, std::string* error = nullptr);

        /// 按当前行填报文对象（不发送）。未知 id 为空。
        std::optional<CigiEntityCtrlV4> entityCtrlPacket(std::uint16_t entityId) const;
        std::optional<CigiEntityPositionCtrlV4> entityPositionPacket(std::uint16_t entityId) const;

    private:
        std::vector<EntityAuthorityRow> _entities;
    };
} // namespace aerovista::sync
