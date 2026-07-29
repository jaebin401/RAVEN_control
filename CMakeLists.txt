cmake_minimum_required(VERSION 3.16)

project(RAVEN_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(WARNING
        "RAVEN_control hardware executables require Linux SocketCAN; "
        "skipping them on ${CMAKE_SYSTEM_NAME}.")
    return()
endif()

find_package(Threads REQUIRED)

function(raven_add_socketcan_executable target source)
    add_executable(${target} ${source})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(${target} PRIVATE Threads::Threads)
endfunction()

raven_add_socketcan_executable(
    motor_id_config
    tools/motor_id_config.cpp
)

raven_add_socketcan_executable(
    position_control_multi
    apps/position_control_multi.cpp
)

add_executable(
    show_xbox_control_data
    tools/show_xbox_control_data.cpp
)
target_compile_options(
    show_xbox_control_data
    PRIVATE -Wall -Wextra -Wpedantic
)
target_link_libraries(
    show_xbox_control_data
    PRIVATE Threads::Threads
)
