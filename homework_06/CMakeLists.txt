add_library(ballistics_06 src/ballistics.cpp)

target_include_directories(ballistics_06 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ballistics_cli src/main.cpp)

target_link_libraries(ballistics_cli PRIVATE ballistics_06)

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

    FetchContent_MakeAvailable(googletest)

    add_executable(ballistics_tests tests/ballistics_tests.cpp)

    target_link_libraries(ballistics_tests PRIVATE ballistics_06 GTest::gtest_main)

    gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)
endif()
