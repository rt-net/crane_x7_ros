# crane_x7_examples

このパッケージはCRANE-X7 ROS 2パッケージのサンプルコード集です。

- [crane\_x7\_examples](#crane_x7_examples)
  - [準備（実機を使う場合）](#準備実機を使う場合)
    - [1. CRANE-X7本体をPCに接続する](#1-crane-x7本体をpcに接続する)
    - [2. USB通信ポートの接続を確認する](#2-usb通信ポートの接続を確認する)
    - [3. move\_groupとcontrollerを起動する](#3-move_groupとcontrollerを起動する)
      - [標準のCRANE-X7を使用する場合](#標準のcrane-x7を使用する場合)
      - [RealSense D435マウンタ搭載モデルを使用する場合](#realsense-d435マウンタ搭載モデルを使用する場合)
  - [準備 (Gazeboを使う場合)](#準備-gazeboを使う場合)
    - [1. move\_groupとGazeboを起動する](#1-move_groupとgazeboを起動する)
  - [準備（Mock Componentsを使う場合）](#準備mock-componentsを使う場合)
    - [1. move\_groupとcontrollerを起動する](#1-move_groupとcontrollerを起動する)
  - [サンプルプログラムを実行する](#サンプルプログラムを実行する)
    - [Gazeboでサンプルプログラムを実行する場合](#gazeboでサンプルプログラムを実行する場合)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
      - [Videos](#videos)
    - [pose\_groupstate](#pose_groupstate)
      - [Videos](#videos-1)
    - [joint\_values](#joint_values)
      - [Videos](#videos-2)
    - [cartesian\_path](#cartesian_path)
      - [Videos](#videos-3)
    - [pick\_and\_place](#pick_and_place)
      - [Videos](#videos-4)
  - [Camera Examples](#camera-examples)
    - [aruco\_detection](#aruco_detection)
      - [Videos](#videos-5)
    - [color\_detection](#color_detection)
      - [Videos](#videos-6)
    - [point\_cloud\_detection](#point_cloud_detection)
      - [Videos](#videos-7)

## 準備（実機を使う場合）

![crane_x7](https://rt-net.github.io/images/crane-x7/CRANE-X7-500x500.png)

### 1. CRANE-X7本体をPCに接続する
CRANE-X7本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

**※CRANE-X7本体が接触しないように、十分なスペースを確保してください。**

### 2. USB通信ポートの接続を確認する

USB通信ポートの設定については`crane_x7_control`の
[README](../crane_x7_control/README.md)
を参照してください。

**正しく設定できていない場合、CRANE-X7が動作しないので注意してください**

### 3. move_groupとcontrollerを起動する

#### 標準のCRANE-X7を使用する場合
次のコマンドでmove_group (`crane_x7_moveit_config`)と
controller (`crane_x7_control`)を起動します。

```sh
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
```

#### RealSense D435マウンタ搭載モデルを使用する場合
[RealSense D435マウンタ](https://github.com/rt-net/crane_x7_Hardware/blob/master/3d_print_parts/v1.0/CRANE-X7_HandA_RealSenseD435マウンタ.stl)を搭載している場合は次のコマンドを実行します。RealSense D435が起動し、camera_linkがロボットモデルに追加されます。

```sh
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true
```

## 準備 (Gazeboを使う場合)

![crane_x7_gazebo](https://rt-net.github.io/images/crane-x7/crane_x7_gazebo_ros2.png)

### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`crane_x7_moveit_config`)と
Gazeboを起動します。

```sh
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

## 準備（Mock Componentsを使う場合）

### 1. move_groupとcontrollerを起動する

次のコマンドでmove_group (`crane_x7_moveit_config`)と
controller (`crane_x7_control`)を起動します。

```sh
ros2 launch crane_x7_examples demo.launch.py use_mock_components:=true
```

Mock Componentsではカメラを使ったサンプルを実行することはできません。

## サンプルプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
ros2 launch crane_x7_examples example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

### Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
ros2 launch crane_x7_examples example.launch.py example:='gripper_control' use_sim_time:='true'
```

## Examples

`demo.launch`を実行している状態で各サンプルを実行できます。

- [gripper_control](#gripper_control)
- [pose_groupstate](#pose_groupstate)
- [joint_values](#joint_values)
- [cartesian_path](#cartesian_path)
- [pick_and_place](#pick_and_place)

実行できるサンプルの一覧は、`example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_x7_examples example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [gripper_control, pose_groupstate, joint_values,pick_and_place, cartesian_path]
        (default: 'pose_groupstate')
```

---

### gripper_control

ハンドを開閉させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='gripper_control'
```

<img src=https://rt-net.github.io/images/crane-x7/gazebo_gripper_example.gif width=500px />

#### Videos
[![crane_x7_gripper_control_demo](http://img.youtube.com/vi/uLRLkwbXUP0/hqdefault.jpg)](https://youtu.be/uLRLkwbXUP0)

[back to example list](#examples)

---

### pose_groupstate

group_stateを使うコード例です。

SRDFファイル[crane_x7_moveit_config/config/crane_x7.srdf](../crane_x7_moveit_config/config/crane_x7.srdf)
に記載されている`home`と`vertical`の姿勢に移行します。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='pose_groupstate'
```

<img src=https://rt-net.github.io/images/crane-x7/gazebo_pose_groupstate.gif width=500px />

#### Videos
[![crane_x7_pose_groupstate_demo](http://img.youtube.com/vi/FH18dA_xcjM/hqdefault.jpg)](https://youtu.be/FH18dA_xcjM)

[back to example list](#examples)

---

### joint_values

アームのジョイント角度を１つずつ変更させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='joint_values'
```
<img src= https://rt-net.github.io/images/crane-x7/gazebo_joint_values_example.gif width = 500px />

#### Videos
[![crane_x7_joint_values_demo](http://img.youtube.com/vi/skRwrrlUl4c/hqdefault.jpg)](https://youtu.be/skRwrrlUl4c)

[back to example list](#examples)

---

### cartesian_path

[Cartesian Path](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html#cartesian-paths)
を生成し、手先で円を描くコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='cartesian_path'
```

#### Videos
[![crane_x7_cartesian_path_demo](http://img.youtube.com/vi/XLhbUqsP2WA/hqdefault.jpg)](https://youtu.be/XLhbUqsP2WA)

[back to example list](#examples)

---

### pick_and_place

モノを掴む・持ち上げる・運ぶ・置くコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='pick_and_place'
```
<img src = https://rt-net.github.io/images/crane-x7/bringup_rviz.gif width = 500px />

**実機を使う場合**

CRANE-X7から20cm離れた位置にピッキング対象を設置します。

<img src = https://rt-net.github.io/images/crane-x7/bringup.jpg width = 500px />

サンプルで使用しているこのオレンジ色のソフトボールはRT ROBOT SHOPの[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)から入手することができます。

#### Videos

[![crane_x7_pick_and_place_demo](http://img.youtube.com/vi/S_MwSvG2tKw/hqdefault.jpg)](https://youtu.be/S_MwSvG2tKw)

[back to example list](#examples)

## Camera Examples

[RealSense D435マウンタ](https://github.com/rt-net/crane_x7_Hardware/blob/master/3d_print_parts/v1.0/CRANE-X7_HandA_RealSenseD435マウンタ.stl)搭載モデルのカメラを使用したサンプルコードです。

[「RealSense D435マウンタ搭載モデルを使用する場合」](#realsense-d435マウンタ搭載モデルを使用する場合)の手順に従って`demo.launch`を実行している状態で各サンプルを実行できます。

- [aruco\_detection](#aruco_detection)
- [color\_detection](#color_detection)
- [point\_cloud\_detection](#point_cloud_detection)

実行できるサンプルの一覧は、`camera_example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_x7_examples camera_example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [aruco_detection, point_cloud_detection]
        (default: 'aruco_detection')
```

### aruco_detection

モノに取り付けたArUcoマーカをカメラで検出し、マーカ位置に合わせて掴むコード例です。
マーカは[aruco_markers.pdf](./aruco_markers.pdf)をA4紙に印刷し、一辺50mmの立方体に取り付けて使用します。

検出されたマーカの位置姿勢はtfのフレームとして配信されます。
tfの`frame_id`はマーカIDごとに異なりID0のマーカの`frame_id`は`target_0`になります。掴む対象は`target_0`に設定されています。マーカ検出には[OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)を使用しています。

次のコマンドを実行します
```sh
ros2 launch crane_x7_examples camera_example.launch.py example:='aruco_detection'
```

#### Videos
[![crane_x7_aruco_detection_demo](http://img.youtube.com/vi/eWzmG_jbTmM/hqdefault.jpg)](https://youtu.be/eWzmG_jbTmM)

[back to camera example list](#camera-examples)

---

### color_detection

特定の色の物体を検出して掴むコード例です。

デフォルトでは青い物体の位置をtfのフレームとして配信します。
tfの`frame_id`は`target_0`です。
色の検出には[OpenCV](https://docs.opencv.org/4.x/db/d8e/tutorial_threshold.html)を使用しています。
検出した物体の距離は深度画像から取得します。

次のコマンドを実行します
```sh
ros2 launch crane_x7_examples camera_example.launch.py example:='color_detection'
```

#### Videos
[![crane_x7_color_detection_demo](http://img.youtube.com/vi/O8lqw7yemAI/hqdefault.jpg)](https://youtu.be/O8lqw7yemAI)

[back to camera example list](#camera-examples)

---

### point_cloud_detection

点群から物体を検出して掴むコード例です。

検出された物体位置はtfのフレームとして配信されます。
tfの`frame_id`は検出された順に`target_0`、`target_1`、`target_2`…に設定されます。掴む対象は`target_0`に設定されています。
物体検出には[Point Cloud Library](https://pointclouds.org/)を使用しています。

次のコマンドを実行します
```sh
ros2 launch crane_x7_examples camera_example.launch.py example:='point_cloud_detection'
```

#### Videos
[![crane_x7_point_cloud_detection_demo](http://img.youtube.com/vi/RgAjxH0CAuk/hqdefault.jpg)](https://youtu.be/RgAjxH0CAuk)

[back to camera example list](#camera-examples)
