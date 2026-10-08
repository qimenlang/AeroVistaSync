# aerovistaSync

多通道同步模块独立库（Host/IG 双端，CIGI V4 数据面 + TCP/UDP 双链路）。
设计基线见 `doc/design/多通道同步/sync模块化设计.md`；协议与行为见 `doc/design/多通道同步/多通道同步模块设计.md`。

## 接入方式

本库提供两种接入形态：

### 1. add_subdirectory（submodule / 工程内）

```cmake
# 依赖 target 需先提供：AeroVistaConfig、cigicl-static（或 cigicl::cigicl）、ws2_32
add_subdirectory(thirdparty/nlohmannJson)
add_subdirectory(thirdparty/config)
add_subdirectory(thirdparty/sync)
target_link_libraries(your_target PRIVATE aerovistaSync)
```

### 2. find_package（独立安装导出）

```cmake
find_package(aerovistaSync REQUIRED)
target_link_libraries(your_target PRIVATE aerovista::aerovistaSync)
```

> 依赖：cigi CCL（`cigicl-static` 或 `cigicl::cigicl`）、`AeroVistaConfig`（其再依赖 nlohmann/json）、Windows `ws2_32`。不依赖 vsg。

## 命名空间

所有类型在 `namespace aerovista::sync`：

```cpp
#include <aerovista/sync/SynchronSystem.h>
#include <aerovista/sync/HostDriver.h>
#include <aerovista/sync/SyncConfig.h>

IgConfig ig;
SyncSystemConfig syncSystem;
loadIgConfig("ig.json", ig, &error);
auto sync = aerovista::sync::SynchronSystem::create();
sync->initialize(std::optional<IgConfig>{ig}, syncSystem);
```

## 快速开始

- viewhost（Host-only）：持 `aerovista::sync::HostDriver`（`HostSync` + `HostDataManager`；中继时另持虚 `IgSync`）。最小接入也可直接持 `HostSync`（`examples/minimal_viewhost.cpp`：`initialize(HostConfig)` + `run`）。
- 独立 IG：用 `loadIgConfig(path, igConfig, err)` 读配置后 `SynchronSystem::create()->initialize(igConfig)`。

## 测试

`tests/` 里的 Catch2 用例只链 `aerovistaSync`（不依赖 vsg / 引擎）。默认 `AEROVISTA_SYNC_BUILD_TESTS=ON`，`ctest` 目标名 `aerovistaSyncTests`。单跑：`aerovistaSyncTests.exe "[TCP-loopback-connect]"`。

仍要引擎场景、相机或实体显隐才能判定的用例留在本仓库 `engine/Tests`（`vsgEngineTests`）。
