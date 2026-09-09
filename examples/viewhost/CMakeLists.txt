# viewhost — MFC Host 宿主示例（viewhost设计.md）。
#
# MFC 需要 MSVC + ATLMFC 组件；clang 无法编译 MFC 资源/头。非 MSVC 直接跳过。
if(NOT MSVC)
    message(STATUS "aerovistaViewHost requires MSVC (MFC); skipped")
    return()
endif()

# MFC 组件检测（ATLMFC include/库）。CI（windows-latest + VS）装有 MFC；本地无 MFC 时干净跳过。
find_package(MFC)
if(NOT MFC_FOUND)
    message(STATUS "aerovistaViewHost requires the MFC (ATLMFC) component; skipped")
    return()
endif()

# CMAKE_MFC_FLAG 为全局 cache 变量（CMake 无 per-target MFC 开关）：1=静态 MFC，2=共享 MFC DLL。
# 本工程仅 viewhost 一个 MFC target，且默认由上层选项关闭，全局影响可接受。
set(CMAKE_MFC_FLAG 2)

add_executable(aerovistaViewHost WIN32
    src/ViewHostApp.cpp
    src/ViewHostDlg.cpp
    src/EntityPropDlg.cpp
    src/HostDriver.cpp
    src/ViewHostMath.cpp
    resources/ViewHost.rc
)

target_include_directories(aerovistaViewHost PRIVATE src resources)

# MFC 程序标准 Unicode 构建。
target_compile_definitions(aerovistaViewHost PRIVATE
    UNICODE
    _UNICODE
    _WIN32_WINNT=0x0A00
    _AFXDLL
)

target_link_libraries(aerovistaViewHost PRIVATE aerovistaSync)

if(MSVC)
    target_compile_options(aerovistaViewHost PRIVATE /utf-8)
endif()

# CMake Ninja 生成器对 MFC Unicode 可执行的入口选择有 bug（CMake gitlab #21202）：
# 默认按窄字符 WinMainCRTStartup 找 WinMain，而 MFC Unicode 程序实际提供 wWinMain，
# 导致 LNK2019: unresolved WinMain（仅此一个符号缺失——MFC 库已由 afx 头 #pragma comment 自动链上）。
# VS 生成器由 CMAKE_MFC_FLAG 正确处理入口，无需此覆盖；故仅 Ninja + MSVC（ci-debug/ci-release）加。
if(CMAKE_GENERATOR MATCHES "Ninja")
    target_link_options(aerovistaViewHost PRIVATE "/ENTRY:wWinMainCRTStartup")
endif()

# 运行时配置随 exe 一并输出（程序按 cwd/exe 目录读 viewhost.json）。
add_custom_command(TARGET aerovistaViewHost POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/viewhost.json
        $<TARGET_FILE_DIR:aerovistaViewHost>/viewhost.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/engine/resources/config/entities.json
        $<TARGET_FILE_DIR:aerovistaViewHost>/entities.json
)
