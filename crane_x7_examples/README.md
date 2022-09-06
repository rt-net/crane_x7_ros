# crane_x7_examples

このパッケージはCRANE-X7 ROS 2パッケージのサンプルコード集です。

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

次のコマンドでmove_group (`crane_x7_moveit_config`)と
controller (`crane_x7_control`)を起動します。

```sh
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
```

## 準備 (Gazeboを使う場合)

![crane_x7_gazebo](https://rt-net.github.io/images/crane-x7/crane_x7_gazebo_ros2.png)

### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`crane_x7_moveit_config`)と
Gazeboを起動します。

```sh
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

## サンプルプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
ros2 launch crane_x7_examples example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

## Examples

`demo.launch`を実行している状態で各サンプルを実行できます。

- [gripper_control](#gripper_control)
- [pose_groupstate](#pose_groupstate)
- [joint_values](#joint_values)
- [cartesian_path](#cartesian_path)
- [pick_and_place](#pick_and_place)

実行できるサンプルの一覧は、`examples.launch.py`にオプション`-s`を付けて実行することで表示できます。

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

[back to example list](#examples)

---

### joint_values

アームのジョイント角度を１つずつ変更させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='joint_values'
```
<img src= https://rt-net.github.io/images/crane-x7/gazebo_joint_values_example.gif width = 500px />

[back to example list](#examples)

---

### cartesian_path

[Cartesian Path](https://moveit.picknik.ai/foxy/doc/move_group_interface/move_group_interface_tutorial.html#cartesian-paths)
を生成し、手先で円を描くコード例です。

次のコマンドを実行します。

```sh
ros2 launch crane_x7_examples example.launch.py example:='cartesian_path'
```

#### Videos

[![](http://img.youtube.com/vi/-Rt3zc3UXMM/sddefault.jpg)](https://youtu.be/-Rt3zc3UXMM)

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

[![crane_x7_pick_and_place_demo](http://img.youtube.com/vi/_8xBgpgMhk8/hqdefault.jpg)](https://youtu.be/_8xBgpgMhk8)

[back to example list](#examples)
