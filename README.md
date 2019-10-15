[English](README.md) | [日本語](README.ja.md)

# crane_x7_ros

[![Build Status](https://travis-ci.org/rt-net/crane_x7_ros.svg?branch=master)](https://travis-ci.org/rt-net/crane_x7_ros)

![crane_x7_gazebo](https://github.com/rt-net/crane_x7_ros/blob/images/images/crane_x7_gazebo.png "crane_x7_gazebo")

ROS Packages for CRANE-X7.

Product page:  
[https://www.rt-net.jp/products/crane-x7](https://www.rt-net.jp/products/crane-x7?lang=en)

ROS Wiki:  
[https://wiki.ros.org/crane_x7](https://wiki.ros.org/crane_x7)

Examples:  
[crane_x7_examples](https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples)

## System Requirements

This package has been developed and tested on ROS Kinectic & Melodic.
Please see below for details.

- ROS Kinetic
  - OS: Ubuntu 16.04.5 LTS
  - ROS Distribution: Kinetic Kame 1.12.14
  - Rviz 1.12.17
  - MoveIt! 0.9.17
  - Gazebo 7.0.0
- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.3
  - Rviz 1.12.16
  - MoveIt! 1.13.3
  - Gazebo 9.0.0

## Installation

### Build from source

- Install ROS environments. Please see [ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu).

- Install this package from source using `git`.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_ros.git
  ```

- Install package dependencies.

  ```bash
  cd ~/catkin_ws/src
  
  # package for crane_x7_gazebo
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
  
  rosdep install -r -y --from-paths --ignore-src crane_x7_ros
  ```

- Build package using `catkin_make`.

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

## Setup Serial Connection

The `crane_x7_control` node communicates with CRANE-X7 via serial port over USB.
Logged user should have read and write access `/dev/ttyUSB0`.

Change permissions on `/dev/ttyUSB0` with command:

```bash
sudo chmod 666 /dev/ttyUSB0
```

## About CRANE-X7 packages

### crane_x7_description

This package defines CRANE-X7 model data includes links and joints.
The MoveIt! package and Gazebo require this package.

### crane_x7_control

This package controls CRANE-X7 using `Dynamixel SDK C++ Libary`.
(The command `rosdep install` installs this library.)
Read and write permissions on `/dev/ttyUSB0` 
are required for communication between the package and X7.

The device name of serial port and parameters of Dynamixel servo motors are listed in `config/crane_x7_control.yaml`.
If this package did not find the serial port, the package switches its control mode to Dummy Joint Mode from Normal Mode,
and republishes taget joint values as servo angle values.
This is useful for debugging of motion control without X7 hardware.

At startup, this package moves the CRANE-X7 to Home Position in 5 seconds.
At shutdown, this package decrease P gains of the servo motors to stop motion safely.

### crane_x7_moveit_config

This MoveIt! packages launch with:

`roslaunch crane_x7_moveit_config demo.launch`

### crane_x7_bringup

This package includes launch files for startup of CRANE-X7.

### crane_x7_examples

This package includes example codes for CRANE-X7.
Please refere to [./crane_x7_examples/README.md](./crane_x7_examples/README.md).

### crane_x7_gazebo

This package includes Gazebo simulation environments for CRANE-X7.

This package launch with:

`roslaunch crane_x7_gazebo crane_x7_with_table.launch`

---

### Proprietary Rights

CRANE-X7 is an arm robot developed by RT Corporation for research purposes. Please read the license information contained in this repository to find out more about licensing. Companies are permitted to use CRANE-X7 and the materials made available here for internal, research and development purposes only. If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going! Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us. 

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS. 
