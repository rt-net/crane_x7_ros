[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/crane_x7_ros/actions/workflows/industrial_ci.yml)

ROS 2 package suite of CRANE-X7.

<img src=https://rt-net.github.io/images/crane-x7/CRANE-X7-500x500.png width=400px/><img src=https://rt-net.github.io/images/crane-x7/crane_x7_gazebo_ros2.png width=400px />

## Table of Contents

- [crane\_x7\_ros](#crane_x7_ros)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS 2 distributions](#supported-ros-2-distributions)
    - [ROS](#ros)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Build from source](#build-from-source)
  - [Quick Start](#quick-start)
  - [Packages](#packages)
  - [ライセンス](#ライセンス)
  - [開発について](#開発について)

## Supported ROS 2 distributions

- [Foxy](https://github.com/rt-net/crane_x7_ros/tree/foxy-devel)
- Humble

### ROS

- [Melodic](https://github.com/rt-net/crane_x7_ros/tree/master)
- [Noetic](https://github.com/rt-net/crane_x7_ros/tree/master)

## Requirements

- CRANE-X7
  - [製品ページ](https://rt-net.jp/products/crane-x7/)
  - [ウェブショップ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3660)
- Linux OS
  - Ubuntu 22.04
- ROS
  - [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## Installation

### Build from source

```sh
# Setup ROS environment
$ source /opt/ros/humble/setup.bash

# Download crane_x7 repositories
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone -b ros2 https://github.com/rt-net/crane_x7_ros.git
$ git clone -b ros2 https://github.com/rt-net/crane_x7_description.git

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

詳細は[crane_x7_examples](./crane_x7_examples/README.md)を参照してください。

## Packages

- crane_x7_control
  - [README](./crane_x7_control/README.md)
  - CRANE-X7を制御するパッケージです
  - USB通信ポートの設定方法をREAMDEに記載してます
- crane_x7_examples
  - [README](./crane_x7_examples/README.md)
  - CRANE-X7のサンプルコード集です
- crane_x7_gazebo
  - [README](./crane_x7_gazebo/README.md)
  - CRANE-X7のGazeboシミュレーションパッケージです
- crane_x7_moveit_config
  - [README](./crane_x7_moveit_config/README.md)
  - CRANE-X7の`moveit2`設定ファイルです
- crane_x7_description (外部パッケージ)
  - [README](https://github.com/rt-net/crane_x7_description/blob/ros2/README.md)
  - CRANE-X7のモデルデータ（xacro）を定義するパッケージです

## ライセンス

(C) 2018 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[crane_x7_description](https://github.com/rt-net/crane_x7_description/tree/ros2)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[crane_x7_description/LICENSE](https://github.com/rt-net/crane_x7_description/blob/ros2/LICENSE)を参照してください。

## 開発について

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](./CONTRIBUTING.md)に従ってください。
