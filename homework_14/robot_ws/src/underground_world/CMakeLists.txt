cmake_minimum_required(VERSION 3.20)
project(underground_world)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellObservation.msg"
  "msg/LocalScan.msg"
  "msg/MoveCommand.msg"
  "msg/EnemyDown.msg"
  "msg/RobotMetrics.msg"
  "msg/RobotResult.msg"
  "msg/StudentStatus.msg"
  "srv/PayloadTrigger.srv"
)

ament_auto_add_library(underground_world_core
  src/scenario_loader.cpp
  src/world_model.cpp
)
target_include_directories(underground_world_core PUBLIC include)

ament_auto_add_executable(underground_world_node src/underground_world_node.cpp)
target_link_libraries(underground_world_node underground_world_core)

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
ament_auto_add_executable(payload_action_node src/payload_action_node.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(underground_world_node "${cpp_typesupport_target}")
target_link_libraries(mission_explorer_node "${cpp_typesupport_target}")
target_link_libraries(payload_action_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(provided_world_contract_test
    test/provided_world_contract_test.cpp
  )
  target_include_directories(provided_world_contract_test PRIVATE include)
  target_link_libraries(provided_world_contract_test underground_world_core)
  target_compile_definitions(provided_world_contract_test
    PRIVATE UNDERGROUND_WORLD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
  launch
  config
)
