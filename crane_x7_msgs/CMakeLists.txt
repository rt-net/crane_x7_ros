cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  rospy
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
    cfg/ServoParameter.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
