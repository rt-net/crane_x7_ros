[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml)

ROS 2でCRANE-X7を動作させるパッケージです。

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
  - [License](#licenses)
  - [Contributing](#contributing)

## Supported ROS 2 distributions

- [Humble Hawksbill](https://github.com/rt-net/crane_x7_ros/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/crane_x7_ros/tree/jazzy)

## Requirements

- CRANE-X7
  - [製品ページ](https://rt-net.jp/products/crane-x7/)
  - [ウェブショップ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3660)
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
  - CRANE-X7を制御するパッケージです
  - USB通信ポートの設定方法をREAMDEに記載しています
- crane_x7_examples
  - [README](./crane_x7_examples/README.md)
  - CRANE-X7のC++サンプルコード集です
- crane_x7_examples_py
  - [README](./crane_x7_examples_py/README.md)
  - CRANE-X7のPythonサンプルコード集です  
- crane_x7_gazebo
  - [README](./crane_x7_gazebo/README.md)
  - CRANE-X7のGazeboシミュレーションパッケージです
- crane_x7_moveit_config
  - [README](./crane_x7_moveit_config/README.md)
  - CRANE-X7の`MoveIt 2`設定ファイルです
- crane_x7_description (外部パッケージ)
  - [README](https://github.com/rt-net/crane_x7_description/blob/ros2/README.md)
  - CRANE-X7のモデルデータ（xacro）を定義するパッケージです

## How to Use Examples

サンプルプログラムは、C++とPythonの両方を用意しています。詳しくは、以下のリンクをご覧ください。

- C++
  - [crane_x7_examples](./crane_x7_examples/README.md)
- Python
  - [crane_x7_examples_py](./crane_x7_examples_py/README.md)

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

各ファイルにライセンスが明記されている場合、そのライセンスに従います。
特に明記がない場合は、Apache License, Version 2.0に基づいて公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[crane_x7_description](https://github.com/rt-net/crane_x7_description/tree/ros2)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[crane_x7_description/LICENSE](https://github.com/rt-net/crane_x7_description/blob/ros2/LICENSE)を参照してください。

## Contributing

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md#contribution-guide-ja)に従ってください。
