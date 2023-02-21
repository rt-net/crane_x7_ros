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

## Supported ROS distributions

- Melodic
- Noetic

### ROS 2

- [Foxy](https://github.com/rt-net/crane_x7_ros/tree/foxy-devel)
- [Humble](https://github.com/rt-net/crane_x7_ros/tree/humble-devel)

## Installation

### Build from source

- Install ROS environments. Please see [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

- Download the packages for CRANE-X7 using `git`.

  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_ros.git
  ```

- Download [crane_x7_description](https://github.com/rt-net/crane_x7_description) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/crane_x7_description/blob/master/LICENSE) applies to the package.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_description.git
  ```

- Install package dependencies.

  ```bash
  cd ~/catkin_ws/src
  rosdep install -r -y --from-paths . --ignore-src
  ```

- Build packages using `catkin_make`.

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### Upgrading to v2.x.x from v1.0.0 or earlier

Please see https://github.com/rt-net/crane_x7_ros/issues/154 for details of differences in the versions.

Update the package with the following commands:

```bash
# Update crane_x7_ros
cd ~/catkin_ws/src/crane_x7_ros
git pull origin master

# Download crane_x7_description package
cd ~/catkin_ws/src
git clone https://github.com/rt-net/crane_x7_description.git
rosdep install -r -y --from-paths . --ignore-src

# Clean up the workspace and rebuild packages
# Note that other ROS packages in the workspace will also be rebuilt.
cd ~/catkin_ws
rm -r build devel
catkin_make
```

## Setup Serial Connection

The `crane_x7_control` node communicates with CRANE-X7 via serial port over USB.
Logged-in user should have read and write access to `/dev/ttyUSB0`.

Change permissions on `/dev/ttyUSB0` with the following command:

```bash
sudo chmod 666 /dev/ttyUSB0
```

## About CRANE-X7 packages

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

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

The crane_x7_ros depends on [crane_x7_description](https://github.com/rt-net/crane_x7_description) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/crane_x7_description/blob/master/LICENSE) applies to the package.
