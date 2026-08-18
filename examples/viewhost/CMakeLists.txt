# viewhost — MFC Host 宿主示例（viewhost设计.md）。
#
# MFC 需要 MSVC + ATLMFC 组件；clang 无法编译 MFC 资源/头。非 MSVC 直接跳过。
if(NOT MSVC)
    message(STATUS "aerovistaViewHost requires MSVC (MFC); skipped")
    return()
endif()

# CMAKE_MFC_FLAG 为全局 cache 变量（CMake 无 per-target MFC 开关）：1=静态 MFC，2=共享 MFC DLL。
# 本工程仅 viewhost 一个 MFC target，且默认由上层选项关闭，全局影响可接受。
set(CMAKE_MFC_FLAG 2)

add_executable(aerovistaViewHost WIN32
    src/ViewHostApp.cpp
    src/ViewHostDlg.cpp
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
)

target_link_libraries(aerovistaViewHost PRIVATE aerovistaSync)

# 运行时配置随 exe 一并输出（程序按 cwd/exe 目录读 viewhost.json）。
add_custom_command(TARGET aerovistaViewHost POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/viewhost.json
        $<TARGET_FILE_DIR:aerovistaViewHost>/viewhost.json
)
