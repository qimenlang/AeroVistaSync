# platform — 模拟平台 Host（MFC；直接持 HostSync，平台同步设计.md §4）。原 viewhost 示例仍在 examples/viewhost。
#
# MFC 需要 MSVC + ATLMFC 组件；clang 无法编译 MFC 资源/头。非 MSVC 直接跳过。
if(NOT MSVC)
    message(STATUS "aerovistaPlatform requires MSVC (MFC); skipped")
    return()
endif()

# MFC 组件检测（ATLMFC include/库）。CI（windows-latest + VS）装有 MFC；本地无 MFC 时干净跳过。
find_package(MFC)
if(NOT MFC_FOUND)
    message(STATUS "aerovistaPlatform requires the MFC (ATLMFC) component; skipped")
    return()
endif()

# CMAKE_MFC_FLAG 为全局 cache 变量（CMake 无 per-target MFC 开关）：1=静态 MFC，2=共享 MFC DLL。
# 与 examples/viewhost 同设为 2；两 target 都构建时值相同。
set(CMAKE_MFC_FLAG 2)

add_executable(aerovistaPlatform WIN32
    src/PlatformApp.cpp
    src/PlatformFrame.cpp
    src/PlatformView.cpp
    src/SceneTreePane.cpp
    src/IgListPane.cpp
    src/PacketProbePane.cpp
    src/CommandPane.cpp
    src/EntityPropDlg.cpp
    src/PlatformMath.cpp
    src/HudLabel.cpp
    resources/Platform.rc
)

target_include_directories(aerovistaPlatform PRIVATE src resources)

# MFC 程序标准 Unicode 构建。
target_compile_definitions(aerovistaPlatform PRIVATE
    UNICODE
    _UNICODE
    _WIN32_WINNT=0x0A00
    _AFXDLL
)

target_link_libraries(aerovistaPlatform PRIVATE aerovistaSync)

if(MSVC)
    target_compile_options(aerovistaPlatform PRIVATE /utf-8)
endif()

# CMake Ninja 生成器对 MFC Unicode 可执行的入口选择有 bug（CMake gitlab #21202）：
# 默认按窄字符 WinMainCRTStartup 找 WinMain，而 MFC Unicode 程序实际提供 wWinMain，
# 导致 LNK2019: unresolved WinMain（仅此一个符号缺失——MFC 库已由 afx 头 #pragma comment 自动链上）。
# VS 生成器由 CMAKE_MFC_FLAG 正确处理入口，无需此覆盖；故仅 Ninja + MSVC（ci-debug/ci-release）加。
if(CMAKE_GENERATOR MATCHES "Ninja")
    target_link_options(aerovistaPlatform PRIVATE "/ENTRY:wWinMainCRTStartup")
endif()

# 运行时配置随 exe 一并输出（程序按 cwd/exe 目录读 platform.json）。
add_custom_command(TARGET aerovistaPlatform POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/platform.json
        $<TARGET_FILE_DIR:aerovistaPlatform>/platform.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/entities.json
        $<TARGET_FILE_DIR:aerovistaPlatform>/entities.json
)
