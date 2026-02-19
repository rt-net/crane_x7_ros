# crane_x7_control

このパッケージは[ros2_control](https://github.com/ros-controls/ros2_control)をベースにした、CRANE-X7 のコントローラパッケージです。

## Table of Contents

- [crane\_x7\_control](#crane_x7_control)
  - [ros2\_control Files](#ros2_control-files)
  - [Setup](#setup)
    - [USB Port Configuration](#usb-port-configuration)
    - [latency_timer Setting](#latency_timer-setting)
    - [Return Delay Time Setting](#return-delay-time-setting)
  - [How to Launch Nodes](#how-to-launch-nodes)
  - [Controller Manager Parameters](#controller-manager-parameters)
    - [Control Cycle](#control-cycle)
    - [Controllers](#controllers)
  - [crane\_x7\_hardware Parameters](#crane_x7_hardware-parameters)
    - [USB Port](#usb-port)
    - [Baudrate](#baudrate)
    - [Communication Timeout](#communication-timeout)
    - [Configuration File Paths for RT Manipulator C++ Library](#configuration-file-paths-for-rt-manipulator-c-library)

## ros2_control Files

- `crane_x7_control::CraneX7Hardware (crane_x7_hardware)`
  - 本パッケージがエクスポートする[Hardware Components](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components)です
  - CRANE-X7実機と通信します
  - crane_x7_descriptionから読み込まれます
- [launch/crane_x7_control.launch.py](./launch/crane_x7_control.launch.py)
  - [Controller Manager](https://control.ros.org/master/doc/getting_started/getting_started.html#controller-manager)とコントローラを起動するlaunchファイルです
- [config/crane_x7_controllers.yaml](./config/crane_x7_controllers.yaml)
  - Controller Managerのパラメータファイルです

## Setup

`crane_x7_hardware`がCRANE-X7実機と通信するために、PCとCRANE-X7の設定が必要です。

### USB Port Configuration

`crane_x7_hardware`はUSB通信ポート（`/dev/ttyUSB*`）を経由してCRANE-X7と通信します。

次のコマンドでアクセス権限を変更します。

```sh
# /dev/ttyUSB0を使用する場合
sudo chmod 666 /dev/ttyUSB0
```

永続的なアクセス権限を付与する場合は次のコマンドを実行します。


```sh
sudo usermod -aG dialout $USER
reboot
```

### latency_timer Setting

CRANE-X7を200 Hz周期で制御するためには、
USB通信ポートとサーボモータの設定を変更します。

下記のコマンドを実行してUSB通信ポートの`latency_timer`を変更します。

参考資料：https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting

```sh
# /dev/ttyUSB0を使用する場合
sudo chmod a+rw /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### Return Delay Time Setting

CRANE-X7に搭載されているサーボモータ[Dynamixel](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)には、`Return Delay Time`というパラメータがあります。

デフォルトは250がセットされており、サーボモータが`Instruction Packet`を受信してから`Status Packet`を送信するまでに`500 usec`の遅れがあります。

[Dynamixel Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)を使用して`Retrun Delay Time`を小さくすると、制御周期が早くなります。

![Setting Return Delay Time](https://rt-net.github.io/images/crane-x7/setting_return_delay_time.png)

## How to Launch Nodes

`crane_x7_control.launch.py`を実行すると、`Controller Manager`ノードが起動し、以下のコントローラが読み込まれます。

- joint_state_controller (`joint_state_controller/JointStateController`)
- crane_x7_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- crane_x7_gripper_controller (`position_controllers/GripperActionController`)

ノードが起動した後、次のコマンドでジョイント角度情報（`joint_states`）を表示できます

```sh
ros2 topic echo /joint_states
```

## Controller Manager Parameters

`Controller Manager`のパラメータは、[config/crane_x7_controllers.yaml](./config/crane_x7_controllers.yaml)で設定しています。

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200  # Hz

    crane_x7_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    crane_x7_gripper_controller:
      type: position_controllers/GripperActionController
    joint_state_controller:
      type: joint_state_controller/JointStateController
```

### Control Cycle

`update_rate`は制御周期を設定します。

### Controllers

CRANE-X7の腕の制御用に`crane_x7_arm_controller`を、グリッパの制御用に`crane_x7_gripper_controller`を設定しています。

## crane_x7_hardware Parameters

`crane_x7_hardware`のパラメータは、`crane_x7_description/urdf/crane_x7.urdf.xacro`で設定しています。

```xml
  <xacro:arg name="port_name" default="/dev/ttyUSB0" />
  <xacro:arg name="baudrate" default="3000000" />
  <xacro:arg name="timeout_seconds" default="1.0" />
  <xacro:arg name="manipulator_config_file_path" default="" />
  <xacro:arg name="manipulator_links_file_path" default="" />
```

### USB Port

`port_name`はCRANE-X7との通信に使用するUSB通信ポートを設定します。

### Baudrate

`baudrate`はCRANE-X7に搭載したDynamixelとの通信ボーレートを設定します。

デフォルト値には`3000000` (3 Mbps)を設定しています。

### Communication Timeout

`timeout_seconds`は通信タイムアウト時間（秒）を設定します。

`crane_x7_hardware`は、一定時間（デフォルト1秒間）通信に失敗し続けると、read/write動作を停止します。
USBケーブルや電源ケーブルが抜けた場合等に有効です。

### Configuration File Paths for RT Manipulator C++ Library

`crane_x7_hardware`は、CRANE-X7と通信するために[RTマニピュレータC++ライブラリ](https://github.com/rt-net/rt_manipulators_cpp)を使用しています。

`manipulatcor_config_file_path`と`manipulator_links_file_path`には、ライブラリが読み込むサーボ設定ファイルとリンク情報ファイルへのパスを設定します。

---

[Back to top](#crane_x7_control)
