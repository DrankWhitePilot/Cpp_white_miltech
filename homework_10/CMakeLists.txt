cmake_minimum_required(VERSION 3.22)
project(homework_10_multithreaded LANGUAGES CXX)

find_package(Threads REQUIRED)

add_executable(mission_multithreaded
    src/main.cpp
    src/types.cpp
    src/json_target_provider.cpp
    src/file_config_loader.cpp
    src/analytical_solver.cpp
    src/model_math.cpp
    src/ballistic_table.cpp
    src/table_solver.cpp
    src/drone_state.cpp
    src/factory.cpp
    src/mission_processor.cpp
    src/thread_safe_target_provider.cpp
    src/drone_physics.cpp
)

target_include_directories(mission_multithreaded
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(mission_multithreaded
    PRIVATE
        -Wall
        -Wextra
        -pedantic
)


target_link_libraries(mission_multithreaded
    PRIVATE
        Threads::Threads
)
