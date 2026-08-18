// 最小接入示例：viewhost（Host-only）如何复用 aerovistaSync。
//
// viewhost 场景：独立 Host 进程，无渲染，只等 IG 连接 + 发命令。
//   1. 用 sync 库的 loadHostConfig 读取独立配置文件（只含 hostConfig）；
//   2. 直接持 HostSync（传输层），initialize + run；
//   3. 按目标 fps 驱动 update 扇出（无渲染节拍）。
//
// Host 采样/扇出不经过 SynchronSystem（那是 IG 决策器）；纯 Host 直发走 HostSync。
// 构建：add_subdirectory(aerovistaSync) 后链接。

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <aerovista/sync/HostSync.h>
#include <aerovista/sync/SyncConfig.h>

using aerovista::sync::HostConfig;
using aerovista::sync::HostSync;
using aerovista::sync::loadHostConfig;

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

    // 2) 纯 Host：直接持 HostSync（传输层），initialize 起 accept/UDP 线程 + run 置 RUNNING。
    HostSync hostSync;
    if (!hostSync.initialize(host))
    {
        std::cerr << "HostSync initialize failed\n";
        return 1;
    }
    hostSync.run();

    std::cout << "[viewhost] Host waiting on UDP "
              << host.udpPortRecv << " / TCP " << host.tcpPort << "\n";

    // 3) 无渲染节拍：按 60fps 驱动 update 扇出（HostSync 线程只收，扇出需外部驱动）。
    constexpr double kFrameMs = 16.667;
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 600; ++frame)
    {
        const double elapsedMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        hostSync.update(elapsedMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    hostSync.shutdown();
    return 0;
}
