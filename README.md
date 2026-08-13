# aerovistaSync

多通道同步模块独立库（Host/IG 双端，CIGI V4 数据面 + TCP/UDP 双链路）。
设计基线见 `doc/design/sync模块化设计.md`；协议与行为见 `doc/design/多通道同步模块设计.md`。

## 接入方式

本库提供两种接入形态：

### 1. add_subdirectory（submodule / 工程内）

```cmake
# 依赖 target 需先提供：vsg::vsg、cigicl-static（或 cigicl::cigicl）、ws2_32
add_subdirectory(thirdparty/sync)
target_link_libraries(your_target PRIVATE aerovistaSync)
```

### 2. find_package（独立安装导出）

```cmake
find_package(aerovistaSync REQUIRED)
target_link_libraries(your_target PRIVATE aerovista::aerovistaSync)
```

> 依赖：vsg（`find_package(vsg)` 或由使用方提供 `vsg::vsg`）、cigi CCL（`cigicl-static` 或 `cigicl::cigicl`）、Windows `ws2_32`。

## 命名空间

所有类型在 `namespace aerovista::sync`：

```cpp
#include <aerovista/sync/SynchronSystem.h>
#include <aerovista/sync/SyncConfig.h>

aerovista::sync::SyncRoleConfig role;
role.enableHost = true;
role.hostConfig = /* loadHostConfig(...) 或直接填 */;
auto sync = aerovista::sync::SynchronSystem::create();
sync->initialize(role);
```

## 快速开始

- viewhost（Host-only）：见 `examples/minimal_viewhost.cpp` + `examples/viewhost.json`。
- 独立 IG：用 `loadIgConfig(path, igConfig, err)` 读配置后 `initialize(enableIg=true)`。

## 测试

sync 库的行为测试目前在本项目的 `engine/Tests`（`[viewhost]` / `[standalone]` / `[clock]` 等标签）。
