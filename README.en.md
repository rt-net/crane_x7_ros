[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml)

This is a ROS 2 package suite for the CRANE-X7.

<img src=https://rt-net.github.io/images/crane-x7/CRANE-X7-500x500.png width=400px/><img src=https://rt-net.github.io/images/crane-x7/crane_x7_gazebo_ros2.png width=400px />

## Table of Contents

- [crane\_x7\_ros](#crane_x7_ros)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS distributions](#supported-ros-distributions)
  - [Requirements](#requirements)
  - [Installation](#installation)
  - [Quick Start](#quick-start)
  - [Packages](#packages)
  - [How to Use Examples](#how-to-use-examples)
  - [License](#license)
  - [Contributing](#contributing)

## Supported ROS 2 distributions

- [Humble Hawksbill](https://github.com/rt-net/crane_x7_ros/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/crane_x7_ros/tree/jazzy)

## Requirements

- CRANE-X7
  - [Product Introduction](https://rt-net.jp/products/crane-x7/)
  - [Web Shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3660&language=en)
- Linux OS
  - Ubuntu 24.04
- ROS
  - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)

## Installation

### Source Build

```sh
# Download crane_x7 repositories
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Quick Start

```sh
# Connect CRANE-X7 to PC, then
source ~/ros2_ws/install/setup.bash
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch crane_x7_examples example.launch.py example:='gripper_control'

# Press [Ctrl-c] to terminate.
```

## Packages

- crane_x7_control
  - [README](./crane_x7_control/README.md)
  - This package provides a hardware driver for CRANE-X7.
  - The procedure for configuring the USB communication port is described in the README.
- crane_x7_examples
  - [README](./crane_x7_examples/README.md)
  - This package provides C++ example code for CRANE-X7.
- crane_x7_examples_py
  - [README](./crane_x7_examples_py/README.md)
  - This package provides Python example code for CRANE-X7.
- crane_x7_gazebo
  - [README](./crane_x7_gazebo/README.md)
  - This package provides Gazebo simulation environments for CRANE-X7.
- crane_x7_moveit_config
  - [README](./crane_x7_moveit_config/README.md)
  - This package provides configuration files for `MoveIt 2`.
- crane_x7_description (external package)
  - [README](https://github.com/rt-net/crane_x7_description/blob/ros2/README.en.md)
  - This package provides model data (xacro) of CRANE-X7.

## How to Use Examples

Sample programs are available in both C++ and Python. See the links below for details.

- C++
  - [crane_x7_examples](./crane_x7_examples/README.md)
- Python
  - [crane_x7_examples_py](./crane_x7_examples_py/README.md)

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

crane_x7_ros depends on [crane_x7_description](https://github.com/rt-net/crane_x7_description/tree/ros2) package.
RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/crane_x7_description/blob/ros2/LICENSE) applies to the package.

## Contributing

- This software is open source, but its development is not open.
- This software is essentially provided as open source software on an “AS IS” (in its current state) basis.
- No free support is available for this software.
- Requests for bug fixes and corrections of typographical errors are always accepted; however, requests for additional features will be subject to our internal guidelines. For further details, please refer to the [Contribution Guidelines](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md#contribution-guide-en).
