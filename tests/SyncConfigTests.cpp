#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/SyncConfig.h>

#include "Common.h"

#include <string>

using aerovista::sync::loadHostConfig;
using aerovista::sync::loadIgConfig;

// =============================================================================
// loadHostConfig：viewhost / 独立 Host 进程的 sync 库入口（sync模块化设计.md §4.0）
// =============================================================================

TEST_CASE("loadHostConfig parses a host-only config file", "[unit][config][sync][host][CFG-host-parse-ok]")
{
    const TempConfigFile file(R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE(cfg.udpPortRecv == 8000);
    REQUIRE(cfg.tcpPort == 8100);
    REQUIRE_FALSE(cfg.relay.enable);
    REQUIRE_FALSE(cfg.igConfig.has_value());
}

TEST_CASE("loadHostConfig rejects unknown top-level keys", "[unit][config][sync][host][CFG-host-reject-unknown]")
{
    const TempConfigFile file(R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, "bogus": 1 })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadHostConfig rejects partial hostConfig object", "[unit][config][sync][host][CFG-host-reject-partial]")
{
    const TempConfigFile file(R"({ "hostConfig": { "udpPortRecv": 8000 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadHostConfig leaves relay off when enable is false even if igConfig is present",
          "[unit][config][sync][host][CFG-host-relay-off]")
{
    const TempConfigFile file(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": false, "expectedIgCount": 3 }, )"
        R"("igConfig": { "udpPortRecv": 8002, "targetAddr": "10.0.0.5", )"
        R"("targetTcpPort": 9100, "targetUdpPortRecv": 9000 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(cfg.relay.enable);
    REQUIRE_FALSE(cfg.igConfig.has_value());
}

TEST_CASE("loadHostConfig reads virtual IG from igConfig when relay is enabled",
          "[unit][config][sync][host][CFG-host-relay-on]")
{
    const TempConfigFile file(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true, "expectedIgCount": 3 }, )"
        R"("igConfig": { "udpPortRecv": 8002, "targetAddr": "10.0.0.5", )"
        R"("targetTcpPort": 9100, "targetUdpPortRecv": 9000 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE(cfg.relay.enable);
    REQUIRE(cfg.relay.expectedIgCount == 3);
    REQUIRE(cfg.igConfig.has_value());
    REQUIRE(cfg.igConfig->udpPortRecv == 8002);
    REQUIRE(cfg.igConfig->target.addr == "10.0.0.5");
    REQUIRE(cfg.igConfig->target.tcpPort == 9100);
    REQUIRE(cfg.igConfig->target.udpPortRecv == 9000);
}

TEST_CASE("loadHostConfig rejects relay enable without igConfig",
          "[unit][config][sync][host][CFG-host-relay-need-ig]")
{
    const TempConfigFile missing(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true, "expectedIgCount": 1 } })");
    const TempConfigFile partial(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true, "expectedIgCount": 1 }, )"
        R"("igConfig": { "udpPortRecv": 8002 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(missing.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
    error.clear();
    REQUIRE_FALSE(loadHostConfig(partial.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadHostConfig rejects relay enable without expectedIgCount",
          "[unit][config][sync][host][CFG-host-relay-need-count]")
{
    const TempConfigFile file(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true }, )"
        R"("igConfig": { "udpPortRecv": 8002, "targetAddr": "10.0.0.5", )"
        R"("targetTcpPort": 9100, "targetUdpPortRecv": 9000 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadHostConfig rejects relay expectedIgCount below one",
          "[unit][config][sync][host][CFG-host-relay-count]")
{
    const TempConfigFile zero(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true, "expectedIgCount": 0 }, )"
        R"("igConfig": { "udpPortRecv": 8002, "targetAddr": "10.0.0.5", )"
        R"("targetTcpPort": 9100, "targetUdpPortRecv": 9000 } })");
    const TempConfigFile frac(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true, "expectedIgCount": 1.5 }, )"
        R"("igConfig": { "udpPortRecv": 8002, "targetAddr": "10.0.0.5", )"
        R"("targetTcpPort": 9100, "targetUdpPortRecv": 9000 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(zero.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
    error.clear();
    REQUIRE_FALSE(loadHostConfig(frac.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadHostConfig rejects relay when virtual IG UDP port matches Host",
          "[unit][config][sync][host][CFG-host-relay-port]")
{
    const TempConfigFile file(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": true, "expectedIgCount": 1 }, )"
        R"("igConfig": { "udpPortRecv": 8000, "targetAddr": "10.0.0.5", )"
        R"("targetTcpPort": 9100, "targetUdpPortRecv": 9000 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadHostConfig rejects unknown keys inside relay",
          "[unit][config][sync][host][CFG-host-relay-unknown]")
{
    const TempConfigFile file(
        R"({ "hostConfig": { "udpPortRecv": 8000, "tcpPort": 8100 }, )"
        R"("relay": { "enable": false, "bogus": 1 } })");
    HostConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadHostConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

// =============================================================================
// loadIgConfig：独立 IG 进程 / 外部引擎的 sync 库入口（sync模块化设计.md §4.1）
// =============================================================================

TEST_CASE("loadIgConfig parses an ig-only config file", "[unit][config][sync][ig][CFG-ig-parse-ok]")
{
    const TempConfigFile file(
        R"({ "igConfig": { "udpPortRecv": 8005, )"
        R"("targetAddr": "127.0.0.1", "targetTcpPort": 8100, "targetUdpPortRecv": 8000 } })");
    IgConfig cfg;
    std::string error;
    REQUIRE(loadIgConfig(file.path(), cfg, &error));
    REQUIRE(cfg.udpPortRecv == 8005);
    REQUIRE(cfg.target.addr == "127.0.0.1");
    REQUIRE(cfg.target.tcpPort == 8100);
    REQUIRE(cfg.target.udpPortRecv == 8000);
}

TEST_CASE("loadIgConfig rejects unknown top-level keys", "[unit][config][sync][ig][CFG-ig-reject-unknown]")
{
    const TempConfigFile file(
        R"({ "igConfig": { "udpPortRecv": 8005, )"
        R"("targetAddr": "127.0.0.1", "targetTcpPort": 8100, "targetUdpPortRecv": 8000 }, )"
        R"("hostConfig": {} })");
    IgConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadIgConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("loadIgConfig rejects partial igConfig object", "[unit][config][sync][ig][CFG-ig-reject-partial]")
{
    const TempConfigFile file(
        R"({ "igConfig": { "udpPortRecv": 8005, )"
        R"("targetAddr": "127.0.0.1", "targetTcpPort": 8100 } })");
    IgConfig cfg;
    std::string error;
    REQUIRE_FALSE(loadIgConfig(file.path(), cfg, &error));
    REQUIRE_FALSE(error.empty());
}
