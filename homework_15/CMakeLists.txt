cmake_minimum_required(VERSION 3.22)
project(homework_15_results_uploader LANGUAGES CXX)

find_package(Threads REQUIRED)

add_executable(results_uploader
    src/main.cpp
)

target_include_directories(results_uploader
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../homework_10/include
)

target_compile_options(results_uploader
    PRIVATE
        -Wall
        -Wextra
        -pedantic
)

target_link_libraries(results_uploader
    PRIVATE
        Threads::Threads
)
