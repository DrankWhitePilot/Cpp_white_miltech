add_library(telemetry
    src/telemetry.cpp
)

target_include_directories(telemetry
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(telemetry_check
    src/main.cpp
)

target_link_libraries(telemetry_check
    PRIVATE
        telemetry
)
