cmake_minimum_required(VERSION 3.22)
project(HW17_mavlink_udp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES gpiod)
check_cxx_source_compiles([[
#include <gpiod.h>
int main()
{
    auto* chip = gpiod_chip_open_by_name("gpiochip0");
    (void)chip;
    return 0;
}
]] HW17_GPIOD_V1_API)
unset(CMAKE_REQUIRED_LIBRARIES)

add_library(hw17_hardware_transport
    src/uart_link.cpp
    src/gpio_controller.cpp
    src/mavlink_udp.cpp
)

target_include_directories(hw17_hardware_transport
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(hw17_hardware_transport
    SYSTEM PUBLIC
        ${CMAKE_SOURCE_DIR}/c_library_v2
)

target_compile_options(hw17_hardware_transport PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(hw17_hardware_transport PUBLIC gpiod)

if(HW17_GPIOD_V1_API)
    target_compile_definitions(hw17_hardware_transport PUBLIC HW17_GPIOD_V1=1)
endif()

add_library(hw17_math_core
    src/types.cpp
    src/model_math.cpp
    src/ballistic_table.cpp
    src/table_solver.cpp
    src/drone_state.cpp
)

target_include_directories(hw17_math_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw17_math_core PRIVATE -Wall -Wextra -pedantic)

add_library(hw17_mission_processor
    src/mission_processor.cpp
)

target_include_directories(hw17_mission_processor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw17_mission_processor PRIVATE -Wall -Wextra -pedantic)

target_compile_definitions(hw17_mission_processor
    PRIVATE
        HW17_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

target_link_libraries(hw17_mission_processor PUBLIC hw17_math_core)

add_library(hw17_drone_control
    src/drone_controller.cpp
)

target_include_directories(hw17_drone_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw17_drone_control PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(hw17_drone_control PUBLIC hw17_math_core)

add_executable(mission_mavlink_udp
    src/uart_main.cpp
)

target_link_libraries(mission_mavlink_udp
    PRIVATE
        hw17_hardware_transport
        hw17_mission_processor
        hw17_drone_control
        Threads::Threads
)

target_compile_options(mission_mavlink_udp PRIVATE -Wall -Wextra -pedantic)

include(CTest)

if(BUILD_TESTING)
    add_executable(hw17_mission_processor_test
        tests/mission_processor_test.cpp
    )

    target_compile_definitions(hw17_mission_processor_test
        PRIVATE
            HW17_TEST_DATA_FILE="${CMAKE_CURRENT_SOURCE_DIR}/data/ballistic_table.txt"
    )

    target_compile_options(hw17_mission_processor_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(hw17_mission_processor_test PRIVATE hw17_mission_processor)

    add_test(NAME hw17_mission_processor_test COMMAND hw17_mission_processor_test)

    add_executable(hw17_drone_controller_test
        tests/drone_controller_test.cpp
    )

    target_compile_options(hw17_drone_controller_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(hw17_drone_controller_test PRIVATE hw17_drone_control)

    add_test(NAME hw17_drone_controller_test COMMAND hw17_drone_controller_test)

    add_executable(hw17_uart_link_test
        tests/uart_link_test.cpp
        src/uart_link.cpp
    )

    target_include_directories(hw17_uart_link_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_compile_options(hw17_uart_link_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(hw17_uart_link_test PRIVATE util)

    add_test(NAME hw17_uart_link_test COMMAND hw17_uart_link_test)

    add_executable(hw17_mavlink_udp_test
        tests/mavlink_udp_test.cpp
    )

    target_compile_options(hw17_mavlink_udp_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(hw17_mavlink_udp_test PRIVATE hw17_hardware_transport)

    add_test(NAME hw17_mavlink_udp_test COMMAND hw17_mavlink_udp_test)

    add_executable(hw17_mavlink_no_ack_test
        tests/mavlink_no_ack_test.cpp
    )

    target_compile_options(hw17_mavlink_no_ack_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(hw17_mavlink_no_ack_test PRIVATE hw17_hardware_transport)

    add_test(NAME hw17_mavlink_no_ack_test COMMAND hw17_mavlink_no_ack_test)

    add_executable(hw17_checker_driver
        tests/checker_driver.cpp
    )

    target_compile_options(hw17_checker_driver PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(hw17_checker_driver PRIVATE hw17_hardware_transport)
endif()
