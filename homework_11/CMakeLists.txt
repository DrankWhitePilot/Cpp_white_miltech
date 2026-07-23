cmake_minimum_required(VERSION 3.22)
project(homework_11_uart_gpio LANGUAGES CXX)

find_package(Threads REQUIRED)

add_executable(mission_uart_drop
    src/uart_main.cpp
    src/types.cpp
    src/model_math.cpp
    src/uart_link.cpp
    src/gpio_controller.cpp
    src/drone_controller.cpp
)

target_include_directories(mission_uart_drop
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(mission_uart_drop
    PRIVATE
        -Wall
        -Wextra
        -pedantic
)

target_link_libraries(mission_uart_drop
    PRIVATE
        Threads::Threads
        gpiod
)
