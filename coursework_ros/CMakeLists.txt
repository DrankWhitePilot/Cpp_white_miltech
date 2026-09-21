cmake_minimum_required(VERSION 3.16)
project(coursework_guidance_ros LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(AIRSIM_ROOT "/home/user/AirSim-1.8.1" CACHE PATH "AirSim source tree")
option(COURSEWORK_REQUIRE_AIRSIM "Fail the build when AirSim C++ dependencies are missing" OFF)

add_library(coursework_math_core
  ../homework_10/src/types.cpp
  ../homework_10/src/model_math.cpp
)
target_include_directories(coursework_math_core PUBLIC ../homework_10/include)
target_compile_options(coursework_math_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(guidance_node src/guidance_node.cpp)
target_link_libraries(guidance_node coursework_math_core)
ament_target_dependencies(guidance_node geometry_msgs nav_msgs rclcpp)
target_compile_options(guidance_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(guidance_checker src/guidance_checker.cpp)
ament_target_dependencies(guidance_checker geometry_msgs nav_msgs rclcpp)
target_compile_options(guidance_checker PRIVATE -Wall -Wextra -Wpedantic)

add_executable(target_trajectory_node src/target_trajectory_node.cpp)
target_include_directories(target_trajectory_node PRIVATE ../homework_10/include)
ament_target_dependencies(target_trajectory_node geometry_msgs rclcpp)
target_compile_options(target_trajectory_node PRIVATE -Wall -Wextra -Wpedantic)

set(AIRSIM_HEADER "${AIRSIM_ROOT}/AirLib/include/api/RpcLibClientBase.hpp")
set(AIRSIM_LIB_DIR "${AIRSIM_ROOT}/build_coursework/output/lib")
if(EXISTS "${AIRSIM_HEADER}" AND
  EXISTS "${AIRSIM_LIB_DIR}/libAirLib.a" AND
  EXISTS "${AIRSIM_LIB_DIR}/librpc.a" AND
  EXISTS "${AIRSIM_LIB_DIR}/libMavLinkCom.a")
  add_executable(airsim_online_node
    src/airsim_online_node.cpp
    ../homework_11/src/drone_controller.cpp
  )
  target_include_directories(airsim_online_node PRIVATE
    ../homework_10/include
    ../homework_11/include
    ${AIRSIM_ROOT}/AirLib/include
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/MavLinkCom/include
    ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include
    ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/dependencies/include
  )
  target_link_libraries(airsim_online_node
    coursework_math_core
    ${AIRSIM_LIB_DIR}/libAirLib.a
    ${AIRSIM_LIB_DIR}/librpc.a
    ${AIRSIM_LIB_DIR}/libMavLinkCom.a
    pthread
  )
  ament_target_dependencies(airsim_online_node geometry_msgs nav_msgs rclcpp)
  target_compile_definitions(airsim_online_node PRIVATE MSGPACK_PP_VARIADICS_MSVC=0)
  target_compile_options(airsim_online_node PRIVATE -Wall -Wextra -Wpedantic)
else()
  if(COURSEWORK_REQUIRE_AIRSIM)
    message(FATAL_ERROR "AirSim C++ headers or static libraries are missing under ${AIRSIM_ROOT}")
  endif()
  message(WARNING "AirSim C++ headers or libraries not found at ${AIRSIM_ROOT}; airsim_online_node is disabled")
endif()

install(TARGETS
  guidance_node
  guidance_checker
  target_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

if(TARGET airsim_online_node)
  install(TARGETS airsim_online_node DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY
  launch
  config
  scenarios
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ../homework_10/data/targets.json
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
