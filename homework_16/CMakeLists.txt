cmake_minimum_required(VERSION 3.20)
project(homework_16_i2c LANGUAGES CXX)

add_executable(hw16_i2c_sensor
    src/main.cpp
)

target_compile_options(hw16_i2c_sensor
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
