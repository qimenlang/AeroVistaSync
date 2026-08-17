# aerovistaSync — 多通道同步模块独立库。
# 依赖方向单向：宿主 → aerovistaSync；库不反向依赖引擎。
# namespace: aerovista::sync（sync模块化设计.md §1.2）。
#
# 接入方式：
#   1. add_subdirectory(thirdparty/sync)      （本工程 / submodule 形态）
#   2. find_package(aerovistaSync)            （独立 repo 安装导出后）

cmake_minimum_required(VERSION 3.20)
project(aerovistaSync VERSION 0.1.0 LANGUAGES CXX)

# --- 依赖声明 ---
# vsg：本工程由 thirdparty/vsg submodule 提供 vsg::vsg；独立 repo 用 find_package(vsg)。
if(NOT TARGET vsg::vsg)
    find_package(vsg REQUIRED)
endif()

# cigicl：本工程由 thirdparty/cigi 提供 cigicl-static；独立 repo 用 find_package(cigicl)。
if(NOT TARGET cigicl-static AND NOT TARGET cigicl::cigicl)
    find_package(cigicl REQUIRED)
endif()

if(TARGET cigicl-static)
    set(AEROVISTA_SYNC_CIGI_TARGET cigicl-static)
elseif(TARGET cigicl::cigicl)
    set(AEROVISTA_SYNC_CIGI_TARGET cigicl::cigicl)
endif()

# 头文件源（供 IDE / install 引用）
set(AEROVISTA_SYNC_PUBLIC_HEADERS
    include/aerovista/sync/CigiIncludes.h
    include/aerovista/sync/CigiWire.h
    include/aerovista/sync/EventProcess.h
    include/aerovista/sync/HostSync.h
    include/aerovista/sync/IgSync.h
    include/aerovista/sync/SyncConfig.h
    include/aerovista/sync/SyncJson.h
    include/aerovista/sync/SyncMath.h
    include/aerovista/sync/SyncProtocol.h
    include/aerovista/sync/SynchronSystem.h
    include/aerovista/sync/UdpSocket.h
)

add_library(aerovistaSync STATIC
    ${AEROVISTA_SYNC_PUBLIC_HEADERS}
    src/CigiWire.cpp
    src/EventProcess.cpp
    src/HostSync.cpp
    src/IgSync.cpp
    src/SyncConfig.cpp
    src/SynchronSystem.cpp
    src/UdpSocket.cpp
)

target_include_directories(aerovistaSync PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(aerovistaSync
    PUBLIC
        ${AEROVISTA_SYNC_CIGI_TARGET}
        ws2_32
    PRIVATE
        vsg::vsg
)

target_compile_features(aerovistaSync PUBLIC cxx_std_17)

# --- 可安装 / 导出（独立 repo find_package 用；本工程内默认关闭） ---
# 依赖 cigicl-static 不在本工程 export set，工程内开启会报错。独立 repo 时
# 由 repo 自己的 cigi 依赖（find_package 导出的 cigicl::cigicl）配合。
option(AEROVISTA_SYNC_INSTALL "Install aerovistaSync (independent repo usage)" OFF)
if(AEROVISTA_SYNC_INSTALL)
    include(GNUInstallDirs)
    install(TARGETS aerovistaSync
        EXPORT aerovistaSyncTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES ${AEROVISTA_SYNC_PUBLIC_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aerovista/sync)
    install(EXPORT aerovistaSyncTargets
        FILE aerovistaSyncTargets.cmake
        NAMESPACE aerovista::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aerovistaSync
    )
endif()

# 可选：最小接入示例（默认关闭）。供其他 vsg 项目参考接入契约。
option(AEROVISTA_BUILD_SYNC_EXAMPLE "Build aerovistaSync minimal integration example" OFF)
if(AEROVISTA_BUILD_SYNC_EXAMPLE)
    add_executable(aerovistaSyncExample
        examples/minimal_viewhost.cpp
    )
    target_link_libraries(aerovistaSyncExample PRIVATE aerovistaSync)
endif()
