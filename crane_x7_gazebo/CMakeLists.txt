cmake_minimum_required(VERSION 3.5)
project(crane_x7_gazebo)

find_package(ament_cmake REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  gazebo_ros_control
)

install(DIRECTORY
  launch
  worlds
  gui
  DESTINATION share/${PROJECT_NAME}/
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()