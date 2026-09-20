cmake_minimum_required(VERSION 3.22)
project(HW11_HW11_uart_gpio LANGUAGES CXX)

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
]] HW11_GPIOD_V1_API)
unset(CMAKE_REQUIRED_LIBRARIES)

add_library(hw11_le_hardware_transport
    src/uart_link.cpp
    src/gpio_controller.cpp
)

target_include_directories(hw11_le_hardware_transport
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw11_le_hardware_transport PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(hw11_le_hardware_transport PUBLIC gpiod)

if(HW11_GPIOD_V1_API)
    target_compile_definitions(hw11_le_hardware_transport PUBLIC HW11_GPIOD_V1=1)
endif()

add_library(hw11_le_math_core
    src/types.cpp
    src/model_math.cpp
    src/ballistic_table.cpp
    src/table_solver.cpp
    src/drone_state.cpp
)

target_include_directories(hw11_le_math_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw11_le_math_core PRIVATE -Wall -Wextra -pedantic)

add_library(hw11_le_mission_processor
    src/mission_processor.cpp
)

target_include_directories(hw11_le_mission_processor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw11_le_mission_processor PRIVATE -Wall -Wextra -pedantic)

target_compile_definitions(hw11_le_mission_processor
    PRIVATE
        HW11_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

target_link_libraries(hw11_le_mission_processor PUBLIC hw11_le_math_core)

add_library(hw11_le_drone_control
    src/drone_controller.cpp
)

target_include_directories(hw11_le_drone_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(hw11_le_drone_control PRIVATE -Wall -Wextra -pedantic)

target_link_libraries(hw11_le_drone_control PUBLIC hw11_le_math_core)

add_executable(mission_uart_drop
    src/uart_main.cpp
)

target_link_libraries(mission_uart_drop
    PRIVATE
        hw11_le_hardware_transport
        hw11_le_mission_processor
        hw11_le_drone_control
        Threads::Threads
)

target_compile_options(mission_uart_drop PRIVATE -Wall -Wextra -pedantic)

include(CTest)

if(BUILD_TESTING)
    add_executable(mission_processor_test
        tests/mission_processor_test.cpp
    )

    target_compile_definitions(mission_processor_test
        PRIVATE
            HW11_TEST_DATA_FILE="${CMAKE_CURRENT_SOURCE_DIR}/data/ballistic_table.txt"
    )

    target_compile_options(mission_processor_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(mission_processor_test PRIVATE hw11_le_mission_processor)

    add_test(NAME mission_processor_test COMMAND mission_processor_test)

    add_executable(drone_controller_test
        tests/drone_controller_test.cpp
    )

    target_compile_options(drone_controller_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(drone_controller_test PRIVATE hw11_le_drone_control)

    add_test(NAME drone_controller_test COMMAND drone_controller_test)

    add_executable(uart_link_test
        tests/uart_link_test.cpp
        src/uart_link.cpp
    )

    target_include_directories(uart_link_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_compile_options(uart_link_test PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(uart_link_test PRIVATE util)

    add_test(NAME uart_link_test COMMAND uart_link_test)
endif()
