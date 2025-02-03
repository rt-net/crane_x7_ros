[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml)

ROS 2 package suite of CRANE-X7.

<img src=https://rt-net.github.io/images/crane-x7/CRANE-X7-500x500.png width=400px/><img src=https://rt-net.github.io/images/crane-x7/crane_x7_gazebo_ros2.png width=400px />

## Table of Contents

- [crane\_x7\_ros](#crane_x7_ros)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS 2 distributions](#supported-ros-2-distributions)
    - [ROS 1](#ros-1)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Build from source](#build-from-source)
  - [Quick Start](#quick-start)
  - [Packages](#packages)
  - [License](#license)

## Supported ROS 2 distributions

- [Foxy](https://github.com/rt-net/crane_x7_ros/tree/foxy-devel)
- [Humble](https://github.com/rt-net/crane_x7_ros/tree/humble)
- [Jazzy](https://github.com/rt-net/crane_x7_ros/tree/jazzy)

### ROS 1

- [Melodic](https://github.com/rt-net/crane_x7_ros/tree/master)
- [Noetic](https://github.com/rt-net/crane_x7_ros/tree/master)

## Requirements

- CRANE-X7
  - [Product Introduction](https://rt-net.jp/products/crane-x7/)
  - [Web Shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3660&language=en)
- Linux OS
  - Ubuntu 24.04
- ROS
  - [Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

## Installation

### Build from source

```sh
# Download crane_x7 repositories
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
$ git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

# Install dependencies
$ rosdep install -r -y -i --from-paths .

# Build & Install
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## Quick Start

```sh
# Connect CRANE-X7 to PC, then
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# Terminal 2
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch crane_x7_examples example.launch.py example:='gripper_control'

# Press [Ctrl-c] to terminate.
```

Please refer to [crane_x7_examples](./crane_x7_examples/README.md) for details.

## Packages

- crane_x7_control
  - [README](./crane_x7_control/README.md)
  - This package includes a hardware driver for CRANE-X7.
- crane_x7_examples
  - [README](./crane_x7_examples/README.md)
  - This package includes C++ example codes for CRANE-X7.
- crane_x7_examples_py
  - [README](./crane_x7_examples_py/README.md)
  - This package includes Python example codes for CRANE-X7.
- crane_x7_gazebo
  - [README](./crane_x7_gazebo/README.md)
  - This package includes Gazebo simulation environments for CRANE-X7.
- crane_x7_moveit_config
  - [README](./crane_x7_moveit_config/README.md)
  - This package includes configuration files for `moveit2`.
- crane_x7_description (external package)
  - [README](https://github.com/rt-net/crane_x7_description/blob/ros2/README.en.md)
  - This package includes a model data (xacro) of CRANE-X7.

---

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

The crane_x7_ros depends on [crane_x7_description](https://github.com/rt-net/crane_x7_description/tree/ros2) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/crane_x7_description/blob/ros2/LICENSE) applies to the package.
