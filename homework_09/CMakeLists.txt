add_executable(mission_smart_state
    src/main.cpp
    src/json_target_provider.cpp
    src/file_config_loader.cpp
    src/analytical_solver.cpp
    src/ballistic_table.cpp
    src/table_solver.cpp
    src/drone_state.cpp
    src/factory.cpp
    src/mission_processor.cpp
)

target_include_directories(mission_smart_state
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(mission_smart_state
    PRIVATE
        -Wall
        -Wextra
        -pedantic
)
