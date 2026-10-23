set(TARGET_NAME aerovistaSyncTests)

add_executable(${TARGET_NAME}
    Testutil.h
    TcpSocketTests.cpp
    UdpSocketTests.cpp
    RttTrackerTests.cpp
    HostDataManagerTests.cpp
    ViewHostMathTests.cpp
    SyncClockInjectTests.cpp
    SyncWireTests.cpp
    HostIGLinkTests.cpp
    HostDriverTests.cpp
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
