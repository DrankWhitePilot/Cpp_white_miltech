add_executable(mission_stl
    src/main.cpp
    src/json_target_provider.cpp
    src/file_config_loader.cpp
    src/analytical_solver.cpp
    src/factory.cpp
    src/mission_processor.cpp
)

target_include_directories(mission_stl
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(mission_stl
    PRIVATE
        -Wall
        -Wextra
        -pedantic
)
