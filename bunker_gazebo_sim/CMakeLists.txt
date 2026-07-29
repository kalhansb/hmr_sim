cmake_minimum_required(VERSION 3.8)
project(bunker_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

