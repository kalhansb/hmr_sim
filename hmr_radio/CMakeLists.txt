cmake_minimum_required(VERSION 3.5)

project(hmr_radio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Propagation core: one implementation shared by the node, the CLI and the tests
add_library(radio_core STATIC
  src/radio_core.cpp
)
target_include_directories(radio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(radio_core PUBLIC cxx_std_17)
set_target_properties(radio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(radio_core PUBLIC yaml-cpp)

# ROS2 comms emulator with the v2 propagation stack (runs as node name hmr_comms_sim)
add_executable(hmr_radio_node
  src/hmr_radio_node.cpp
)
ament_target_dependencies(hmr_radio_node
  rclcpp
  std_msgs
  nav_msgs
)
target_link_libraries(hmr_radio_node radio_core)

# Offline probe: gamma_explicit and per-link breakdowns without ROS
add_executable(radio_probe
  src/radio_probe.cpp
)
target_link_libraries(radio_probe radio_core)

install(TARGETS
  hmr_radio_node
  radio_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/extract_radio_world.py
  scripts/link_breakdown_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_radio_core test/test_radio_core.cpp)
  target_link_libraries(test_radio_core radio_core)
  target_compile_definitions(test_radio_core PRIVATE HMR_RADIO_SRC="${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
