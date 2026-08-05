cmake_minimum_required(VERSION 3.16)

project(RAVEN_control VERSION 0.2.0 LANGUAGES CXX)

option(
    RAVEN_BUILD_ROS2_BRIDGE
    "Build the read-only RS02 feedback to ROS 2 joint-state bridge"
    ON
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(yaml-cpp QUIET)

set(RAVEN_HAS_YAML_CPP 0)
set(RAVEN_YAML_CPP_TARGET "")
if(TARGET yaml-cpp::yaml-cpp)
    set(RAVEN_HAS_YAML_CPP 1)
    set(RAVEN_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(RAVEN_HAS_YAML_CPP 1)
    set(RAVEN_YAML_CPP_TARGET yaml-cpp)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR
        "yaml-cpp is required for RAVEN_control on Linux. "
        "Install libyaml-cpp-dev and configure again.")
else()
    message(WARNING
        "yaml-cpp was not found. Pure configuration validation and "
        "safety tests can build, but loading YAML is unavailable.")
endif()

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
target_compile_definitions(
    raven_safety
    PRIVATE RAVEN_HAS_YAML_CPP=${RAVEN_HAS_YAML_CPP}
)
if(RAVEN_YAML_CPP_TARGET)
    target_link_libraries(
        raven_safety
        PUBLIC ${RAVEN_YAML_CPP_TARGET}
    )
endif()

add_library(
    raven_hal
    STATIC
    src/config/motor_config.cpp
    src/hal/motor_driver.cpp
    src/hal/rs02_operation_feedback.cpp
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
target_compile_definitions(
    raven_hal
    PRIVATE RAVEN_HAS_YAML_CPP=${RAVEN_HAS_YAML_CPP}
)

add_library(
    raven_telemetry
    STATIC
    src/telemetry/joint_state_aggregator.cpp
)
target_include_directories(
    raven_telemetry
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(raven_telemetry PUBLIC raven_hal)
raven_enable_warnings(raven_telemetry)

add_library(
    raven_logging
    STATIC
    src/logging/motion_logger.cpp
)
target_include_directories(
    raven_logging
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
raven_enable_warnings(raven_logging)

add_library(
    raven_dynamics
    STATIC
    src/dynamics/gravity_compensator.cpp
)
target_include_directories(
    raven_dynamics
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
raven_enable_warnings(raven_dynamics)

add_executable(
    gravity_model_check
    tools/gravity_model_check.cpp
)
target_link_libraries(
    gravity_model_check
    PRIVATE raven_dynamics
)
raven_enable_warnings(gravity_model_check)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")

function(raven_copy_runtime_config local_name example_name)
    set(local_path "${CMAKE_CURRENT_SOURCE_DIR}/config/${local_name}")
    if(EXISTS "${local_path}")
        set(source_path "${local_path}")
    else()
        set(source_path
            "${CMAKE_CURRENT_SOURCE_DIR}/config/${example_name}")
    endif()
    configure_file(
        "${source_path}"
        "${CMAKE_CURRENT_BINARY_DIR}/config/${local_name}"
        COPYONLY
    )
endfunction()

raven_copy_runtime_config(
    joint_limits.yaml
    joint_limits.example.yaml
)
raven_copy_runtime_config(
    motor_config.yaml
    motor_config.example.yaml
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

    add_executable(
        motor_config_test
        tests/motor_config_test.cpp
    )
    target_link_libraries(motor_config_test PRIVATE raven_hal)
    raven_enable_warnings(motor_config_test)
    add_test(NAME motor_config_test COMMAND motor_config_test)

    add_executable(
        motion_logger_test
        tests/motion_logger_test.cpp
    )
    target_link_libraries(motion_logger_test PRIVATE raven_logging)
    raven_enable_warnings(motion_logger_test)
    add_test(NAME motion_logger_test COMMAND motion_logger_test)

    add_executable(
        gravity_compensator_test
        tests/gravity_compensator_test.cpp
    )
    target_link_libraries(
        gravity_compensator_test
        PRIVATE raven_dynamics
    )
    raven_enable_warnings(gravity_compensator_test)
    add_test(
        NAME gravity_compensator_test
        COMMAND gravity_compensator_test
    )

    add_executable(
        joint_state_aggregator_test
        tests/joint_state_aggregator_test.cpp
    )
    target_link_libraries(
        joint_state_aggregator_test
        PRIVATE raven_telemetry
    )
    raven_enable_warnings(joint_state_aggregator_test)
    add_test(
        NAME joint_state_aggregator_test
        COMMAND joint_state_aggregator_test
    )
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_sources(raven_hal PRIVATE src/hal/can_interface.cpp)

    if(RAVEN_BUILD_ROS2_BRIDGE)
        find_package(ament_cmake QUIET)
        find_package(rclcpp QUIET)
        find_package(sensor_msgs QUIET)
        if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND)
            add_executable(
                raven_joint_state_bridge
                apps/ros2/joint_state_bridge.cpp
            )
            target_link_libraries(
                raven_joint_state_bridge
                raven_telemetry
                raven_hal
            )
            ament_target_dependencies(
                raven_joint_state_bridge
                rclcpp
                sensor_msgs
            )
            raven_enable_warnings(raven_joint_state_bridge)
        else()
            message(WARNING
                "ROS 2 was not found in the current environment. "
                "raven_joint_state_bridge will not be built. "
                "Source /opt/ros/<distro>/setup.bash and configure again.")
        endif()
    endif()

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
        motor_gain_tuner
        tools/motor_gain_tuner.cpp
    )
    target_link_libraries(motor_gain_tuner PRIVATE raven_hal)
    raven_enable_warnings(motor_gain_tuner)

    add_executable(
        position_control_multi
        apps/debug/position_control_multi.cpp
    )
    target_link_libraries(position_control_multi PRIVATE raven_hal)
    raven_enable_warnings(position_control_multi)

    add_executable(
        coordinated_motion_demo
        apps/coordinated_motion_demo.cpp
    )
    target_link_libraries(
        coordinated_motion_demo
        PRIVATE raven_hal raven_logging raven_dynamics
    )
    raven_enable_warnings(coordinated_motion_demo)

    add_executable(
        scripted_motion_demo
        apps/scripted_motion_demo.cpp
    )
    target_link_libraries(
        scripted_motion_demo
        PRIVATE
            raven_hal
            raven_logging
            raven_dynamics
            ${RAVEN_YAML_CPP_TARGET}
    )
    raven_enable_warnings(scripted_motion_demo)

    add_executable(
        joint_characterization
        tests/hardware/joint_characterization.cpp
    )
    target_link_libraries(
        joint_characterization
        PRIVATE
            raven_hal
            raven_logging
            raven_dynamics
            ${RAVEN_YAML_CPP_TARGET}
    )
    raven_enable_warnings(joint_characterization)

    raven_add_linux_executable(
        show_xbox_control_data
        tools/show_xbox_control_data.cpp
    )
else()
    message(STATUS
        "Linux SocketCAN executables are skipped on "
        "${CMAKE_SYSTEM_NAME}.")
endif()
