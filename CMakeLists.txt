cmake_minimum_required(VERSION 3.16)

project(RAVEN_control VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(yaml-cpp QUIET)

function(raven_enable_warnings target)
    target_compile_options(
        ${target}
        PRIVATE -Wall -Wextra -Wpedantic
    )
endfunction()

add_library(
    raven_safety
    STATIC
    src/safety/joint_limiter.cpp
)
target_include_directories(
    raven_safety
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
raven_enable_warnings(raven_safety)

if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(raven_safety PUBLIC yaml-cpp::yaml-cpp)
    target_compile_definitions(raven_safety PRIVATE RAVEN_HAS_YAML_CPP=1)
elseif(TARGET yaml-cpp)
    target_link_libraries(raven_safety PUBLIC yaml-cpp)
    target_compile_definitions(raven_safety PRIVATE RAVEN_HAS_YAML_CPP=1)
else()
    target_compile_definitions(raven_safety PRIVATE RAVEN_HAS_YAML_CPP=0)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        message(FATAL_ERROR
            "yaml-cpp is required for RAVEN_control on Linux. "
            "Install libyaml-cpp-dev and configure again.")
    else()
        message(WARNING
            "yaml-cpp was not found. Pure safety tests can build, but "
            "loading config/joint_limits.yaml is unavailable.")
    endif()
endif()

add_library(
    raven_hal
    STATIC
    src/hal/motor_driver.cpp
)
target_include_directories(
    raven_hal
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
    raven_hal
    PUBLIC raven_safety Threads::Threads
)
raven_enable_warnings(raven_hal)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
configure_file(
    config/joint_limits.yaml
    config/joint_limits.yaml
    COPYONLY
)

include(CTest)
if(BUILD_TESTING)
    add_executable(
        joint_limiter_test
        tests/joint_limiter_test.cpp
    )
    target_link_libraries(joint_limiter_test PRIVATE raven_safety)
    raven_enable_warnings(joint_limiter_test)
    add_test(NAME joint_limiter_test COMMAND joint_limiter_test)

    add_executable(
        motor_driver_safety_test
        tests/motor_driver_safety_test.cpp
    )
    target_link_libraries(motor_driver_safety_test PRIVATE raven_hal)
    raven_enable_warnings(motor_driver_safety_test)
    add_test(
        NAME motor_driver_safety_test
        COMMAND motor_driver_safety_test
    )
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_sources(raven_hal PRIVATE src/hal/can_interface.cpp)

    function(raven_add_linux_executable target source)
        add_executable(${target} ${source})
        target_link_libraries(${target} PRIVATE Threads::Threads)
        raven_enable_warnings(${target})
    endfunction()

    raven_add_linux_executable(
        motor_id_config
        tools/motor_id_config.cpp
    )

    add_executable(
        position_control_multi
        apps/debug/position_control_multi.cpp
    )
    target_link_libraries(position_control_multi PRIVATE raven_hal)
    raven_enable_warnings(position_control_multi)

    raven_add_linux_executable(
        show_xbox_control_data
        tools/show_xbox_control_data.cpp
    )
else()
    message(STATUS
        "Linux SocketCAN executables are skipped on "
        "${CMAKE_SYSTEM_NAME}.")
endif()
