# crane_x7_control

CRANE-X7のためのパッケージ、 `crane_x7_ros` のcontrolパッケージです。

## 通信ポートの設定

`crane_x7_control`は[U2D2](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3618)(USB-RS485変換ケーブル)を使用してCRANE-X7のモータ制御を行います。   
U2D2はLinuxに接続すると`/dev/ttyUSB0`として認識されるので、次のコマンドでアクセス権限を変更します。

```bash
sudo chmod 666 /dev/ttyUSB0
```

CRANE-X7との接続に`/dev/ttyUSB0`以外を使用したい場合は、`crane_x7_control/config/crane_x7_control.yaml`に定義されているパラメータを変更することで設定することができます。

```
dynamixel_port:
  port_name: "/dev/ttyUSB0"
```

## 制御周期の変更

`crane_x7_control`はデフォルト200Hz周期で制御しています。
制御周期を変更する場合は`crane_x7_control/src/hardware.cpp`を編集してください。

```cpp
#define     CONTROL_HZ   (200)
```

実際に動作している制御周期は次のようにトピックから確認できます。

```sh
$ rostopic hz /crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint/current

subscribed to [/crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint/current]
average rate: 199.928
	min: 0.002s max: 0.008s std dev: 0.00055s window: 190
average rate: 198.470
	min: 0.001s max: 0.014s std dev: 0.00099s window: 387
```

実際の制御周期が200Hzに達せず、100Hz程度に低くなってしまう場合、USB通信ポートの`latency_timer`を変更してください。

```sh
# cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

参考資料：https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting

## ネームスペースとトピック

`crane_x7_control`は`/crane_x7`をルートとするネームスペースにパラメータやトピックを定義します。   
各jointについて次のトピックを配信します。

current：電流値[mA]   
dxl_position：現在角度[360/4096度]   
temp：温度[度]   

## dynamic_reconfigure

`crane_x7_control`は`dynamic_reconfigure`に対応しています。次のコマンドで`rqt_reconfigure`を起動してアクセスすると各Jointのサーボパラメータを変更することができます。

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure](https://rt-net.github.io/images/crane-x7/readme_rqt_reconfigure.png)

各パラメータの詳細についてはROBOTIS公式のXM430およびXM540のサーボマニュアルを参照して下さい。   
- [XM430-W350](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [XM540-W270](http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)

## 制御モード設定

CRANE-X7は位置制御モードと電流制御モードに対応しています。基本設定は位置制御モードになっていますが、複数箇所の設定変更を行うことで電流制御モードへ切り替えることが可能です。   
ハンドの設定を位置制御モードから電流制御モードへ変更する手順を紹介します。

1. **サーボの設定変更**   
サーボモータの`Operating Mode`（Address：11）を`3：位置制御モード`から`0：電流制御モード`へ設定変更する

2. **`crane_x7_control`の設定変更**   
`crane_x7_control/config/crane_x7_control.yaml`のハンド部分について次のように変更する   
```diff
gripper_controller:
-  type: "position_controllers/GripperActionController"
+  type: "effort_controllers/GripperActionController"
  publish_rate: 250
  joint: crane_x7_gripper_finger_a_joint
+  gains:
+    crane_x7_gripper_finger_a_joint: { p: 1.0, i: 0.01, d: 0.1 }
  action_monitor_rate: 10
  state_publish_rate:  100
  stalled_velocity_threshold: 0.01
  goal_tolerance: 0.2
  stall_timeout: 0.3

dynamixel_port:
  port_name: "/dev/ttyUSB0"
  baud_rate: 3000000
  joints:
    - crane_x7_shoulder_fixed_part_pan_joint
    - crane_x7_shoulder_revolute_part_tilt_joint
    - crane_x7_upper_arm_revolute_part_twist_joint
    - crane_x7_upper_arm_revolute_part_rotate_joint
    - crane_x7_lower_arm_fixed_part_joint
    - crane_x7_lower_arm_revolute_part_joint
    - crane_x7_wrist_joint
    - crane_x7_gripper_finger_a_joint
  crane_x7_shoulder_fixed_part_pan_joint: {id: 2, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
  crane_x7_shoulder_revolute_part_tilt_joint: {id: 3, center: 2048, home: 2048, effort_const: 2.79, mode: 3 }
  crane_x7_upper_arm_revolute_part_twist_joint: {id: 4, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
  crane_x7_upper_arm_revolute_part_rotate_joint: {id: 5, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
  crane_x7_lower_arm_fixed_part_joint: {id: 6, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
  crane_x7_lower_arm_revolute_part_joint: {id: 7, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
  crane_x7_wrist_joint: {id: 8, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
-  crane_x7_gripper_finger_a_joint: {id: 9, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
+  crane_x7_gripper_finger_a_joint: {id: 9, center: 2048, home: 2048, effort_const: 1.79, mode: 0 }
```

PIDの設定値はCRANE-X7を制御するROS環境によって特性が異なる可能性があります。   


【電流モードに関する注意】   
電流制御モードは、位置制御モードと異なり、サーボに設定された角度リミットが**無効**になります。   
ユーザー自身で作成されたプログラムに適切な制限動作が備わっていない場合、**本体の損傷や、本体が周囲や作業者に接触、あるいは衝突し、失明や打撲による死亡といった思わぬ重大事故が発生する危険があります。**   
ユーザーの責任において十分に安全に注意した上でご使用下さい。   
当該製品および当ソフトウェアの使用中に生じたいかなる損害も株式会社アールティでは一切の責任を負いかねます。
