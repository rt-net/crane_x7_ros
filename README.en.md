[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/crane_x7_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

![crane_x7_gazebo](https://rt-net.github.io/images/crane-x7/crane_x7_gazebo.png "crane_x7_gazebo")

ROS Packages for CRANE-X7.

Product page:  
[https://www.rt-net.jp/products/crane-x7](https://www.rt-net.jp/products/crane-x7?lang=en)

ROS Wiki:  
[https://wiki.ros.org/crane_x7](https://wiki.ros.org/crane_x7)

Examples:  
[crane_x7_examples](https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples)

## System Requirements

These packages have been developed and tested on ROS Melodic & Noetic.
Please see below for details.

- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.9
  - Rviz 1.13.14
  - MoveIt 1.0.6
  - Gazebo 9.0.0
- ROS Noetic
  - OS: Ubuntu 20.04.1 LTS
  - ROS Distribution: Noetic Ninjemys 1.15.7
  - Rviz 1.14.1
  - MoveIt 1.1.0
  - Gazebo 11.2.0

## Installation

### Build from source

- Install ROS environments. Please see [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

- Download the packages for CRANE-X7 using `git`.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_ros.git
  ```

- Install package dependencies.

  ```bash
  cd ~/catkin_ws/src
  
  rosdep install -r -y --from-paths --ignore-src crane_x7_ros
  ```

- Build packages using `catkin_make`.

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

## Setup Serial Connection

The `crane_x7_control` node communicates with CRANE-X7 via serial port over USB.
Logged-in user should have read and write access to `/dev/ttyUSB0`.

Change permissions on `/dev/ttyUSB0` with the following command:

```bash
sudo chmod 666 /dev/ttyUSB0
```

## About CRANE-X7 packages

### crane_x7_description

This package defines CRANE-X7 model data including links and joints.
The MoveIt packages and Gazebo require this package.

### crane_x7_control

This package controls CRANE-X7 using `Dynamixel SDK C++ Library`
which can install by `rosdep install` command.
Read and write permissions on `/dev/ttyUSB0` 
are required for communication between the package and CRANE-X7.

The device name of serial port and parameters of Dynamixel servo motors are listed in `config/crane_x7_control.yaml`.
If this package did not find the serial port, the package switches its control mode to Dummy Joint Mode from Normal Mode
and republishes target joint values as servo angle values.
This is useful for debugging of motion control without CRANE-X7 hardware.

At startup, this package moves the CRANE-X7 to Home Position in 5 seconds.
At shutdown, this package decreases P gains of the servo motors to stop motion safely.

### crane_x7_moveit_config

This package includes configuration files for MoveIt.

To launch the MoveIt demonstration with Rviz:

`roslaunch crane_x7_moveit_config demo.launch`

### crane_x7_bringup

This package includes launch files for startup of CRANE-X7.

### crane_x7_examples

This package includes example codes for CRANE-X7.
Please refer to [./crane_x7_examples/README.md](./crane_x7_examples/README.md).

### crane_x7_gazebo

This package includes Gazebo simulation environments for CRANE-X7.

To simulate CRANE-X7 on the table:

`roslaunch crane_x7_gazebo crane_x7_with_table.launch`

---

### Proprietary Rights

CRANE-X7 is an arm robot developed by RT Corporation for research purposes. Please read the license information contained in this repository to find out more about licensing. Companies are permitted to use CRANE-X7 and the materials made available here for internal, research and development purposes only. If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going! Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us. 

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS. 
