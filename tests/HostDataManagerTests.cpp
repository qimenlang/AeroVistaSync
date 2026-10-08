// HostDataManager 实体权威表单元测试。
//
// 编码 viewhost设计.md §4.0 / sync模块化设计.md §3.4 / 实体与运动控制设计.md §3.2 / §5 / §11
// （目录完整 schema 见 实体管理设计.md §4）：
//   建表 ENT-04-table-*；运行期更新 ENT-04-update-*；按行组包 ENT-04-pack-*。
// 不测 flush / UI。

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/HostDataManager.h>

#include "CigiBaseEntityCtrl.h"
#include "CigiBaseEntityPositionCtrl.h"

#include "Common.h"

#include <cstdint>
#include <string>

using aerovista::sync::EntityAuthorityPose;
using aerovista::sync::EntityAuthorityRow;
using aerovista::sync::EntityAuthorityState;
using aerovista::sync::HostDataManager;

namespace
{
    const EntityAuthorityRow* findRow(const std::vector<EntityAuthorityRow>& rows, std::uint16_t entityId)
    {
        for (const EntityAuthorityRow& row : rows)
        {
            if (row.entityId == entityId)
                return &row;
        }
        return nullptr;
    }

    bool poseIsZero(const EntityAuthorityPose& pose)
    {
        return pose.lat == 0.0 && pose.lon == 0.0 && pose.alt == 0.0 && pose.yawDeg == 0.0 && pose.pitchDeg == 0.0 &&
               pose.rollDeg == 0.0;
    }
} // namespace

TEST_CASE("HostDataManager builds entity rows from initialEntityState",
          "[unit][sync][host-data][ENT-04-table-init]")
{
    const TempConfigFile file(
        R"({ "entities": [)"
        R"({ "id": 1, "name": "teapot_center", "model": "models/teapot.vsgt" },)"
        R"({ "id": 2, "name": "truck", "model": "models/truck.vsgt", "initialEntityState": "Standby" })"
        R"(] })");
    HostDataManager manager;
    std::string error;

    REQUIRE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(error.empty());

    const std::vector<EntityAuthorityRow> rows = manager.entitySnapshot();
    REQUIRE(rows.size() == 2);

    const EntityAuthorityRow* first = findRow(rows, 1);
    REQUIRE(first != nullptr);
    REQUIRE(first->name == "teapot_center");
    REQUIRE(first->model == "models/teapot.vsgt");
    REQUIRE(first->entityState == EntityAuthorityState::ACTIVE);
    REQUIRE(first->alpha == 255);
    REQUIRE(poseIsZero(first->pose));

    const EntityAuthorityRow* second = findRow(rows, 2);
    REQUIRE(second != nullptr);
    REQUIRE(second->entityState == EntityAuthorityState::STANDBY);
    REQUIRE(second->alpha == 255);
    REQUIRE(poseIsZero(second->pose));
}

TEST_CASE("HostDataManager defaults entity name from model basename",
          "[unit][sync][host-data][ENT-04-table-name]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 7, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;

    REQUIRE(manager.loadEntityCatalog(file.path(), &error));
    const auto row = manager.entityRow(7);
    REQUIRE(row.has_value());
    REQUIRE(row->name == "lz.vsgt");
}

TEST_CASE("HostDataManager writes ellipsoid pose into the entity table",
          "[unit][sync][host-data][ENT-04-table-pose]")
{
    const TempConfigFile file(
        R"({ "entities": [ { "id": 1, "model": "models/teapot.vsgt", )"
        R"("pose": { "ellipsoid": { "lla": { "lat": 1.0, "lon": 2.0, "alt": 3.0 }, )"
        R"("eulerYprDeg": [10.0, 20.0, 30.0] } } } ] })");
    HostDataManager manager;
    std::string error;

    REQUIRE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(manager.entitySnapshot().size() == 1);
    const auto row = manager.entityRow(1);
    REQUIRE(row.has_value());
    REQUIRE(row->entityState == EntityAuthorityState::ACTIVE);
    REQUIRE(row->pose.lat == 1.0);
    REQUIRE(row->pose.lon == 2.0);
    REQUIRE(row->pose.alt == 3.0);
    REQUIRE(row->pose.yawDeg == 10.0);
    REQUIRE(row->pose.pitchDeg == 20.0);
    REQUIRE(row->pose.rollDeg == 30.0);
}

TEST_CASE("HostDataManager leaves pose at zero when only local pose is present",
          "[unit][sync][host-data][ENT-04-table-pose-local]")
{
    const TempConfigFile file(
        R"({ "entities": [ { "id": 1, "model": "models/teapot.vsgt", )"
        R"("pose": { "local": { "position": [1.0, 2.0, 3.0], "eulerYprDeg": [4.0, 5.0, 6.0] } } } ] })");
    HostDataManager manager;
    std::string error;

    REQUIRE(manager.loadEntityCatalog(file.path(), &error));
    const auto row = manager.entityRow(1);
    REQUIRE(row.has_value());
    REQUIRE(poseIsZero(row->pose));
}

TEST_CASE("HostDataManager lookup misses unknown entity id",
          "[unit][sync][host-data][ENT-04-table-miss]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE_FALSE(manager.entityRow(99).has_value());
}

TEST_CASE("HostDataManager rejects invalid initialEntityState",
          "[unit][sync][host-data][negative][ENT-04-table-reject-state]")
{
    const TempConfigFile file(
        R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt", "initialEntityState": "Destroyed" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE_FALSE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(error.find("initialEntityState") != std::string::npos);
}

TEST_CASE("HostDataManager rejects duplicate entity id",
          "[unit][sync][host-data][negative][ENT-04-table-reject-dup]")
{
    const TempConfigFile file(
        R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" }, { "id": 1, "model": "models/teapot.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE_FALSE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(error.find("duplicate") != std::string::npos);
}

TEST_CASE("HostDataManager rejects entity id outside 1..65535",
          "[unit][sync][host-data][negative][ENT-04-table-reject-id]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 0, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE_FALSE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(error.find("1..65535") != std::string::npos);
}

TEST_CASE("HostDataManager rejects empty entities array",
          "[unit][sync][host-data][negative][ENT-04-table-reject-empty]")
{
    const TempConfigFile file(R"({ "entities": [] })");
    HostDataManager manager;
    std::string error;
    REQUIRE_FALSE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(error.find("empty") != std::string::npos);
}

TEST_CASE("HostDataManager failed reload leaves existing table unchanged",
          "[unit][sync][host-data][negative][ENT-04-table-reload-keep]")
{
    const TempConfigFile good(R"({ "entities": [ { "id": 4, "name": "keep", "model": "models/lz.vsgt" } ] })");
    const TempConfigFile bad(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" }, { "id": 1, "model": "x.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(good.path(), &error));

    REQUIRE_FALSE(manager.loadEntityCatalog(bad.path(), &error));
    REQUIRE(manager.entitySnapshot().size() == 1);
    REQUIRE(manager.entityRow(4)->name == "keep");
}

TEST_CASE("HostDataManager updates entityState of one row",
          "[unit][sync][host-data][ENT-04-update-state]")
{
    const TempConfigFile file(
        R"({ "entities": [)"
        R"({ "id": 1, "name": "teapot_center", "model": "models/teapot.vsgt" },)"
        R"({ "id": 2, "name": "truck", "model": "models/truck.vsgt", "initialEntityState": "Standby" })"
        R"(] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    REQUIRE(manager.setEntityState(1, EntityAuthorityState::STANDBY, &error));
    REQUIRE(manager.entityRow(1)->entityState == EntityAuthorityState::STANDBY);
    REQUIRE(manager.entityRow(2)->entityState == EntityAuthorityState::STANDBY);

    REQUIRE(manager.setEntityState(2, EntityAuthorityState::ACTIVE, &error));
    REQUIRE(manager.entityRow(1)->entityState == EntityAuthorityState::STANDBY);
    REQUIRE(manager.entityRow(2)->entityState == EntityAuthorityState::ACTIVE);
}

TEST_CASE("HostDataManager updates last pose of one row",
          "[unit][sync][host-data][ENT-04-update-pose]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    const EntityAuthorityPose pose{11.0, 22.0, 33.0, 40.0, 50.0, 60.0};
    REQUIRE(manager.setEntityPose(1, pose, &error));
    const auto row = manager.entityRow(1);
    REQUIRE(row.has_value());
    REQUIRE(row->pose.lat == 11.0);
    REQUIRE(row->pose.lon == 22.0);
    REQUIRE(row->pose.alt == 33.0);
    REQUIRE(row->pose.yawDeg == 40.0);
    REQUIRE(row->pose.pitchDeg == 50.0);
    REQUIRE(row->pose.rollDeg == 60.0);
}

TEST_CASE("HostDataManager updates alpha of one row",
          "[unit][sync][host-data][ENT-04-update-alpha]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    REQUIRE(manager.setEntityAlpha(1, 128, &error));
    REQUIRE(manager.entityRow(1)->alpha == 128);
}

TEST_CASE("HostDataManager rejects runtime update of unknown entity id",
          "[unit][sync][host-data][negative][ENT-04-update-miss]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    REQUIRE_FALSE(manager.setEntityState(99, EntityAuthorityState::STANDBY, &error));
    REQUIRE(error.find("unknown") != std::string::npos);
    REQUIRE(manager.entityRow(1)->entityState == EntityAuthorityState::ACTIVE);

    const EntityAuthorityPose pose{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    REQUIRE_FALSE(manager.setEntityPose(99, pose, &error));
    REQUIRE(error.find("unknown") != std::string::npos);
    REQUIRE(poseIsZero(manager.entityRow(1)->pose));

    REQUIRE_FALSE(manager.setEntityAlpha(99, 1, &error));
    REQUIRE(error.find("unknown") != std::string::npos);
    REQUIRE(manager.entityRow(1)->alpha == 255);
}

TEST_CASE("HostDataManager packs EntityCtrl from the current row",
          "[unit][sync][host-data][ENT-04-pack-ctrl]")
{
    const TempConfigFile file(
        R"({ "entities": [)"
        R"({ "id": 1, "model": "models/teapot.vsgt" },)"
        R"({ "id": 2, "model": "models/truck.vsgt", "initialEntityState": "Standby" })"
        R"(] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    const auto active = manager.entityCtrlPacket(1);
    REQUIRE(active.has_value());
    REQUIRE(active->GetEntityID() == 1);
    REQUIRE(active->GetEntityState() == CigiBaseEntityCtrl::Active);
    REQUIRE(active->GetAlpha() == 255);
    REQUIRE(active->GetEntityType() == 0);

    const auto standby = manager.entityCtrlPacket(2);
    REQUIRE(standby.has_value());
    REQUIRE(standby->GetEntityID() == 2);
    REQUIRE(standby->GetEntityState() == CigiBaseEntityCtrl::Standby);
    REQUIRE(standby->GetAlpha() == 255);
    REQUIRE(standby->GetEntityType() == 0);
}

TEST_CASE("HostDataManager packs EntityCtrl from table after runtime update",
          "[unit][sync][host-data][ENT-04-pack-ctrl-current]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));
    REQUIRE(manager.setEntityState(1, EntityAuthorityState::STANDBY, &error));

    const auto packet = manager.entityCtrlPacket(1);
    REQUIRE(packet.has_value());
    REQUIRE(packet->GetEntityID() == 1);
    REQUIRE(packet->GetEntityState() == CigiBaseEntityCtrl::Standby);
}

TEST_CASE("HostDataManager packs EntityPositionCtrl from the current row",
          "[unit][sync][host-data][ENT-04-pack-pose]")
{
    const TempConfigFile file(
        R"({ "entities": [ { "id": 1, "model": "models/teapot.vsgt", )"
        R"("pose": { "ellipsoid": { "lla": { "lat": 1.0, "lon": 2.0, "alt": 3.0 }, )"
        R"("eulerYprDeg": [10.0, 20.0, 30.0] } } } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    const auto packet = manager.entityPositionPacket(1);
    REQUIRE(packet.has_value());
    REQUIRE(packet->GetEntityID() == 1);
    REQUIRE(packet->GetEntityID() != 0);
    REQUIRE(packet->GetAttachState() == CigiBaseEntityPositionCtrl::Detach);
    REQUIRE(packet->GetLat() == 1.0);
    REQUIRE(packet->GetLon() == 2.0);
    REQUIRE(packet->GetAlt() == 3.0);
    REQUIRE(packet->GetYaw() == Catch::Approx(10.0f));
    REQUIRE(packet->GetPitch() == Catch::Approx(20.0f));
    REQUIRE(packet->GetRoll() == Catch::Approx(30.0f));
}

TEST_CASE("HostDataManager packs EntityPositionCtrl from table after runtime update",
          "[unit][sync][host-data][ENT-04-pack-pose-current]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    const EntityAuthorityPose pose{11.0, 22.0, 33.0, 40.0, 50.0, 60.0};
    REQUIRE(manager.setEntityPose(1, pose, &error));

    const auto packet = manager.entityPositionPacket(1);
    REQUIRE(packet.has_value());
    REQUIRE(packet->GetEntityID() == 1);
    REQUIRE(packet->GetAttachState() == CigiBaseEntityPositionCtrl::Detach);
    REQUIRE(packet->GetLat() == 11.0);
    REQUIRE(packet->GetLon() == 22.0);
    REQUIRE(packet->GetAlt() == 33.0);
    REQUIRE(packet->GetYaw() == Catch::Approx(40.0f));
    REQUIRE(packet->GetPitch() == Catch::Approx(50.0f));
    REQUIRE(packet->GetRoll() == Catch::Approx(60.0f));
}

TEST_CASE("HostDataManager pack misses unknown entity id",
          "[unit][sync][host-data][negative][ENT-04-pack-miss]")
{
    const TempConfigFile file(R"({ "entities": [ { "id": 1, "model": "models/lz.vsgt" } ] })");
    HostDataManager manager;
    std::string error;
    REQUIRE(manager.loadEntityCatalog(file.path(), &error));

    REQUIRE_FALSE(manager.entityCtrlPacket(99).has_value());
    REQUIRE_FALSE(manager.entityPositionPacket(99).has_value());
}
