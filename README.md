[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/crane_x7_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

![crane_x7_gazebo](https://rt-net.github.io/images/crane-x7/crane_x7_gazebo.png "crane_x7_gazebo")

CRANE-X7のROSパッケージです。

製品ページはこちらです。  
[https://www.rt-net.jp/products/crane-x7](https://www.rt-net.jp/products/crane-x7)

ROS Wikiはこちらです。  
[https://wiki.ros.org/crane_x7](https://wiki.ros.org/crane_x7)

ROSのサンプルコード集はこちらです。  
[crane_x7_examples](https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples)

## 動作環境

以下の環境にて動作確認を行っています。

- ROS Kinetic
  - OS: Ubuntu 16.04.5 LTS
  - ROS Distribution: Kinetic Kame 1.12.14
  - Rviz 1.12.17
  - MoveIt! 0.9.17
  - Gazebo 7.0.0
- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.9
  - Rviz 1.13.14
  - MoveIt! 1.0.6
  - Gazebo 9.0.0
- ROS Noetic
  - OS: Ubuntu 20.04.1 LTS
  - ROS Distribution: Noetic Ninjemys 1.15.7
  - Rviz 1.14.1
  - MoveIt! 1.1.0
  - Gazebo 11.2.0

## インストール方法

### ソースからビルドする方法

- [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)を参照しROSをインストールします。

- `git`を使用して本パッケージをダウンロードします。

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_ros.git
  ```

- 依存関係にあるパッケージをインストールします。

  ```bash
  cd ~/catkin_ws/src
  
  # package for crane_x7_gazebo
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
  
  rosdep install -r -y --from-paths --ignore-src crane_x7_ros
  ```

- `catkin_make`を使用して本パッケージをビルドします。

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### `apt`を使用してインストールする方法

後日提供予定です。

## セットアップ方法

`crane_x7_control`が実機と通信する際には`/dev/ttyUSB0`へのアクセス権が必要です。
`/dev/ttyUSB0`へのアクセス権を変更するには下記のコマンドを実行します。

```bash
sudo chmod 666 /dev/ttyUSB0
```

## パッケージ概要

CRANE-X7の各パッケージはcrane_x7_rosにまとめています。  

### crane_x7_description

CRANE-X7のモデルデータやリンクとジョイントの構成を定義するパッケージです。  
MoveIt!やGazeboから呼び出されます。

### crane_x7_control

CRANE-X7の制御を行うパッケージです。  
dynamixel_sdkのC++ライブラリが必要です。  
実機との通信には`/dev/ttyUSB0`へのアクセス権が必要です。

通信に使用するポートの名前やサーボ情報は`config/crane_x7_control.yaml`に記載します。  
設定されたUSBポートが無い場合、コントローラからの指示通りの値を返すダミージョイントモードで動作します。  
ハードウェアを使用しなくてもデバッグが出来るので便利に使って下さい。  

起動時は設定されたホームポジションへ5秒かけて移動します。  
ノードを停止するとサーボをブレーキモードに変更してから終了するので安全に停止することができます。  

### crane_x7_moveit_config

MoveIt!のパッケージです。下記のコマンドで起動します。  

`roslaunch crane_x7_moveit_config demo.launch`

### crane_x7_bringup

CRANE-X7の起動に必要なlaunchファイルをまとめたパッケージです。

### crane_x7_examples

サンプルコード集です。
使い方については[./crane_x7_examples/README.md](./crane_x7_examples/README.md)を参照してください。

### crane_x7_gazebo

GazeboでCRANE-X7のシミュレーションを行うパッケージです。

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

`roslaunch crane_x7_gazebo crane_x7_with_table.launch`

---

### 知的財産権について

CRANE-X7は、アールティが開発した研究用アームロボットです。
このリポジトリのデータ等に関するライセンスについては、LICENSEファイルをご参照ください。
企業による使用については、自社内において研究開発をする目的に限り、本データの使用を許諾します。 
本データを使って自作されたい方は、義務ではありませんが弊社ロボットショップで部品をお買い求めいただければ、励みになります。
商業目的をもって本データを使用する場合は、商業用使用許諾の条件等について弊社までお問合せください。

サーボモータのXM540やXM430に関するCADモデルの使用については、ROBOTIS社より使用許諾を受けています。 
CRANE-X7に使用されているROBOTIS社の部品類にかかる著作権、商標権、その他の知的財産権は、ROBOTIS社に帰属します。

### Proprietary Rights

CRANE-X7 is an arm robot developed by RT Corporation for research purposes. Please read the license information contained in this repository to find out more about licensing. Companies are permitted to use CRANE-X7 and the materials made available here for internal, research and development purposes only. If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going! Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us. 

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS. 
