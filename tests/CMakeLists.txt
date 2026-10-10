set(TARGET_NAME aerovistaSyncTests)

add_executable(${TARGET_NAME}
    Common.h
    TcpSocketTests.cpp
    UdpSocketTests.cpp
    SofRttTrackerTests.cpp
    HostDataManagerTests.cpp
    ViewHostMathTests.cpp
    SyncConfigTests.cpp
    SyncClockInjectTests.cpp
    SyncWireTests.cpp
    HostIGTests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../examples/viewhost/src/ViewHostMath.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../examples/viewhost/src
)

target_link_libraries(${TARGET_NAME}
    PRIVATE
        aerovistaSync
        Catch2::Catch2WithMain
)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
endif()

add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
