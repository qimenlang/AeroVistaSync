// 最小接入示例：viewhost（Host-only）如何复用 aerovistaSync。
//
// viewhost 场景：独立 Host 进程，无渲染，只等 IG 连接 + 发命令。
//   1. 用 sync 库的 loadHostConfig 读取独立配置文件（只含 hostConfig）；
//   2. 构造 SyncRoleConfig{ enableHost=true } 并 initialize；
//   3. 按目标 fps 驱动 postFrame 扇出（无渲染节拍）。
//
// 构建：add_subdirectory(aerovistaSync) 后链接。

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <aerovista/sync/SyncConfig.h>
#include <aerovista/sync/SynchronSystem.h>

using aerovista::sync::HostConfig;
using aerovista::sync::loadHostConfig;
using aerovista::sync::SynchronSystem;
using aerovista::sync::SyncRoleConfig;
using aerovista::sync::SyncSystemConfig;

int main(int argc, char** argv)
{
    const std::string configPath =
        argc > 1 ? argv[1] : "viewhost.json";

    // 1) 独立配置文件 → hostConfig（sync 库内解析，不依赖引擎）
    HostConfig host;
    std::string error;
    if (!loadHostConfig(configPath, host, &error))
    {
        std::cerr << "loadHostConfig failed: " << error << "\n";
        return 1;
    }

    // 2) Host-only 角色
    SyncRoleConfig role;
    role.enableHost = true;
    role.hostConfig = host;

    auto sync = SynchronSystem::create();
    // 纯 Host：装配配置用默认值（Host 不消费 offset/stale 等 IG 侧属性）。
    if (!sync->initialize(role, SyncSystemConfig{}))
        return 1;

    std::cout << "[viewhost] Host waiting on UDP "
              << host.udpPortRecv << " / TCP " << host.tcpPort << "\n";

    // 3) 无渲染节拍：按 60fps 驱动 postFrame 扇出（HostSync 线程已自启，扇出需外部驱动）
    constexpr double kFrameMs = 16.667;
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 600; ++frame)
    {
        const double elapsedMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        sync->postFrame(elapsedMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sync->shutdown();
    return 0;
}
