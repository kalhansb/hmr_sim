cmake_minimum_required(VERSION 3.5)

project(hmr_sim)

find_package(ament_cmake REQUIRED)

# Updates for comms_emulator_helper
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler graphics)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
find_package(ignition-rendering6 REQUIRED)
set(GZ_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})

add_library(HMRNetSim
  SHARED
  src/HMRNetSim.cc
)

target_include_directories(
  HMRNetSim PRIVATE include
)

target_link_libraries(HMRNetSim PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER})

# Skybox system plugin
add_library(SkyboxSystem
  SHARED
  src/SkyboxSystem.cc
)

target_include_directories(
  SkyboxSystem PRIVATE include
)

target_link_libraries(SkyboxSystem PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
  ignition-rendering${GZ_RENDERING_VER}::ignition-rendering${GZ_RENDERING_VER}
)

# World Query System plugin — raycasts a grid of 3D points for GT occupancy + semantic labels
add_library(WorldQuerySystem
  SHARED
  src/WorldQuerySystem.cc
)

target_include_directories(
  WorldQuerySystem PRIVATE include
)

target_link_libraries(WorldQuerySystem PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
  ignition-common${GZ_COMMON_VER}::ignition-common${GZ_COMMON_VER}-graphics
)

# Trajectory recorder plugin — renders RGB-D + semantic frames along a trajectory
add_library(TrajectoryRecorder
  SHARED
  src/TrajectoryRecorder.cc
)

target_include_directories(
  TrajectoryRecorder PRIVATE include
)

target_link_libraries(TrajectoryRecorder PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
)

# ROS2 comms relay node
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hmr_comms_relay_node
  src/hmr_comms_relay_node.cpp
)
ament_target_dependencies(hmr_comms_relay_node
  rclcpp
  std_msgs
)

# ROS2 <-> GZ generic binary bridge node
add_executable(ros2_gz_generic_bridge_node
  src/ros2_gz_generic_bridge_node.cpp
)
ament_target_dependencies(ros2_gz_generic_bridge_node
  rclcpp
)
target_link_libraries(ros2_gz_generic_bridge_node
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
)

install(TARGETS
  HMRNetSim
  SkyboxSystem
  WorldQuerySystem
  TrajectoryRecorder
  hmr_comms_relay_node
  ros2_gz_generic_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# ROS2 build
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
