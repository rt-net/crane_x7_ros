cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_sdk
  roscpp
  rospy
  hardware_interface
  transmission_interface
  crane_x7_msgs
  dynamic_reconfigure
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager dynamixel_sdk
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

find_library( dxl_x64_cpp NAME libdxl_x64_cpp.so PATHS src/dynamixel_sdk/build/linux64 )

add_executable(crane_x7_control src/hardware.cpp src/joint_control.cpp src/dxlport_control.cpp src/control_setting.cpp)

target_link_libraries( crane_x7_control
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS crane_x7_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)

#############
## Testing ##
#############
