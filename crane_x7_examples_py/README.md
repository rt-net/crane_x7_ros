# crane_x7_examples_py

このパッケージはCRANE-X7 ROS 2パッケージのサンプルコード集です。

- [crane\_x7\_examples\_py](#crane_x7_examples_py)
  - [起動方法](#起動方法)
  - [サンプルプログラムを実行する](#サンプルプログラムを実行する)
    - [Gazeboでサンプルプログラムを実行する場合](#gazeboでサンプルプログラムを実行する場合)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
    - [pose\_groupstate](#pose_groupstate)
    - [joint\_values](#joint_values)
    - [pick\_and\_place](#pick_and_place)
  - [Camera Examples](#camera-examples)
    - [aruco\_detection](#aruco_detection)
    - [color\_detection](#color_detection)

## 起動方法
CRANE-X7の起動方法は[crane_x7_examplesのREADME](../crane_x7_examples/README.md)を参照してください。

## サンプルプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
ros2 launch crane_x7_examples_py example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

### Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
ros2 launch crane_x7_examples_py example.launch.py example:='gripper_control' use_sim_time:='true'
```

## Examples

`demo.launch`を実行している状態で各サンプルを実行できます。

- [gripper_control](#gripper_control)
- [pose_groupstate](#pose_groupstate)
- [joint_values](#joint_values)
- [pick_and_place](#pick_and_place)

実行できるサンプルの一覧は、`example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_x7_examples_py example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [gripper_control, pose_groupstate, joint_values, pick_and_place]
        (default: 'pose_groupstate')
```

---

### gripper_control

ハンドを開閉させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples_py example.launch.py example:='gripper_control'
```

<img src=https://rt-net.github.io/images/crane-x7/gazebo_gripper_example.gif width=500px />

[![crane_x7_gripper_control_demo](http://img.youtube.com/vi/uLRLkwbXUP0/hqdefault.jpg)](https://youtu.be/uLRLkwbXUP0)

[back to example list](#examples)

---

### pose_groupstate

group_stateを使うコード例です。

SRDFファイル[crane_x7_moveit_config/config/crane_x7.srdf](../crane_x7_moveit_config/config/crane_x7.srdf)
に記載されている`home`と`vertical`の姿勢に移行します。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples_py example.launch.py example:='pose_groupstate'
```

<img src=https://rt-net.github.io/images/crane-x7/gazebo_pose_groupstate.gif width=500px />

[![crane_x7_pose_groupstate_demo](http://img.youtube.com/vi/FH18dA_xcjM/hqdefault.jpg)](https://youtu.be/FH18dA_xcjM)

[back to example list](#examples)

---

### joint_values

アームのジョイント角度を１つずつ変更させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples_py example.launch.py example:='joint_values'
```
<img src= https://rt-net.github.io/images/crane-x7/gazebo_joint_values_example.gif width = 500px />

[![crane_x7_joint_values_demo](http://img.youtube.com/vi/skRwrrlUl4c/hqdefault.jpg)](https://youtu.be/skRwrrlUl4c)

[back to example list](#examples)

---

### pick_and_place

モノを掴む・持ち上げる・運ぶ・置くコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples_py example.launch.py example:='pick_and_place'
```
<img src = https://rt-net.github.io/images/crane-x7/bringup_rviz.gif width = 500px />

**実機を使う場合**

CRANE-X7から20cm離れた位置にピッキング対象を設置します。

<img src = https://rt-net.github.io/images/crane-x7/bringup.jpg width = 500px />

サンプルで使用しているこのオレンジ色のソフトボールはRT ROBOT SHOPの[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)から入手することができます。

[![crane_x7_pick_and_place_demo](http://img.youtube.com/vi/S_MwSvG2tKw/hqdefault.jpg)](https://youtu.be/S_MwSvG2tKw)

[back to example list](#examples)

## Camera Examples

[RealSense D435マウンタ](https://github.com/rt-net/crane_x7_Hardware/blob/master/3d_print_parts/v1.0/CRANE-X7_HandA_RealSenseD435マウンタ.stl)搭載モデルのカメラを使用したサンプルコードです。

[crane_x7_examplesのREADME](../crane_x7_examples/README.md)に記載されている「RealSense D435マウンタ搭載モデルを使用する場合」の手順に従って`demo.launch`を実行している状態で各サンプルを実行できます。

- [aruco\_detection](#aruco_detection)
- [color\_detection](#color_detection)

実行できるサンプルの一覧は、`camera_example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_x7_examples_py camera_example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [aruco_detection, color_detection]
        (default: 'aruco_detection')
```

### aruco_detection

モノに取り付けたArUcoマーカをカメラで検出し、マーカ位置に合わせて掴むコード例です。
マーカは[aruco_markers.pdf](./aruco_markers.pdf)をA4紙に印刷し、一辺50mmの立方体に取り付けて使用します。

検出されたマーカの位置姿勢はtfのフレームとして配信されます。
tfの`frame_id`はマーカIDごとに異なりID0のマーカの`frame_id`は`target_0`になります。掴む対象は`target_0`に設定されています。マーカ検出には[OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)を使用しています。

次のコマンドを実行します
```sh
ros2 launch crane_x7_examples_py camera_example.launch.py example:='aruco_detection'
```

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
ros2 launch crane_x7_examples_py camera_example.launch.py example:='color_detection'
```

[![crane_x7_color_detection_demo](http://img.youtube.com/vi/O8lqw7yemAI/hqdefault.jpg)](https://youtu.be/O8lqw7yemAI)

[back to camera example list](#camera-examples)

---
