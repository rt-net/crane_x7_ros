# crane_x7_examples

CRANE-X7のためのパッケージ、 `crane_x7` で用いるサンプルをまとめたパッケージです。

## システムの起動方法

CRANE-X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_bringup`の`demo.launch`を起動します。
このlaunchファイルには次のオプションが用意されています。

- fake_execution (default: true)

実機を使用する/使用しない

### シミュレータを使う場合

実機無しで動作を確認する場合、
制御信号ケーブルを接続しない状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

### 実機を使う場合

実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```


## サンプルの実行方法

`demo.launch`を実行している状態で各サンプルを実行することができます。


### gripper_action_example.pyの実行

ハンドを開閉させるコード例です。
このサンプルは実機動作のみに対応しています。

次のコマンドで45度まで開いて閉じる動作を実行します。

```sh
rosrun crane_x7_examples gripper_action_example.py
```


### pose_groupstate_example.pyの実行

group_stateを使うコード例です。

SRDFファイル[crane_x7_moveit_config/config/crane_x7.srdf](../crane_x7_moveit_config/config/crane_x7.srdf)
に記載されている`home`と`vertical`の姿勢に移行します。

次のコマンドを実行します。

```sh
rosrun crane_x7_examples pose_groupstate_example.py
```

### crane_x7_pick_and_place_demo.pyの実行

モノを掴む・持ち上げる・運ぶ・置くコード例です。

次のコマンドを実行します。

```sh
rosrun crane_x7_examples crane_x7_pick_and_place_demo.py
```

![bringup_rviz](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup_rviz.gif "bringup_rviz")

**実機を使う場合**

CRANE-X7から20cm離れた位置にピッキング対象を設置します。

![bringup](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup.jpg "bringup")

サンプルで使用しているこのオレンジ色のソフトボールはRT ROBOT SHOPの[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)から入手することができます。

動作させると[こちら](https://youtu.be/_8xBgpgMhk8)のような動きになります。


### preset_pid_gain_example.pyの実行

`crane_x7_control`の`preset_reconfigure`を使うコード例です。
サーボモータのPIDゲインを一斉に変更できます。

プリセットは[crane_x7_control/scripts/preset_reconfigure.py](../crane_x7_control/scripts/preset_reconfigure.py)
にて編集できます。

次のコマンドを実行すると、`preset_reconfigure.py`と`preset_pid_gain_example.py`のノードを起動します。

```sh
roslaunch crane_x7_examples preset_pid_gain_example.launch
```

### teaching_example.pyの実行

ティーチングのコード例です。X7のPIDゲインを小さくすることでダイレクトティーチングができます。

次のコマンドでノードを起動します。

```sh
roslaunch crane_x7_examples teaching_example.launch
```

以下のキー割当を参考に、キーボードから操作してください。

**Teaching Mode**

起動時のモードです。トルクOFF*状態です。

| キー | 機能 |
----|----
| s / S | 現在の姿勢を保存 |
| d / D | これまでに保存した姿勢を削除 |
| m / M | **Action Mode**へ遷移 |
| q / Q | シャットダウン |


**Action Mode**

Teaching Modeから遷移します。トルクON*状態です。

| キー | 機能 |
----|----
| p / P | 保存した姿勢を１つずつ再生 |
| a / A | 保存した姿勢のすべてを連続再生 |
| l / L | ループ再生 ON / OFF |
| m / M | **Teaching Mode**へ遷移 |
| q / Q | シャットダウン |

- トルクのON / OFFはサーボモータのPIDゲインに小さい値をプリセットすることで実現しています。

### joystick_example.pyの実行

ジョイスティックでX7を動かすコード例です。
手先の位置・姿勢の変更、グリッパーの開閉、PIDゲインのプリセット、ティーチングができます。

ジョイスティックをPCに接続し、`/dev/input/js0`が存在することを確認してください。

次のコマンドでノードを起動します。

#### 実機を使う場合

```sh
roslaunch crane_x7_examples joystick_example.launch
```

#### シミュレータを使う場合

シミュレータを使う場合は、エラーを防ぐため`sim`オプションを追加してください。

```sh
roslaunch crane_x7_examples joystick_example.launch sim:=true
```

#### キー割り当ての変更

デフォルトのキー割り当てはこちらです。ジョイスティックは
[Logicool Wireless Gamepad F710](https://support.logicool.co.jp/ja_jp/product/wireless-gamepad-f710)
を使っています。
![key_config](https://github.com/rt-net/crane_x7_ros/blob/images/images/joystick_example_key_config.png "key_config")

[crane_x7_example/launch/joystick_example.launch](./launch/joystick_example.launch)
のキー番号を編集することで、キー割り当てを変更できます。

```xml
 <node name="joystick_example" pkg="crane_x7_examples" type="joystick_example.py" required="true" output="screen">
    <!-- 使用するジョイスティックコントローラに合わせてvalueを変更してください -->
    <!-- ひとつのボタンに複数の機能を割り当てています -->
    <param name="button_shutdown_1" value="8" type="int" />
    <param name="button_shutdown_2" value="9" type="int" />

    <param name="button_name_enable" value="7" type="int" />
    <param name="button_name_home"  value="8" type="int" />

    <param name="button_preset_enable" value="7" type="int" />
    <param name="button_preset_no1" value="9" type="int" />
```

デフォルトのキー番号はこちらです。
![key_numbers](https://github.com/rt-net/crane_x7_ros/blob/images/images/joystick_example_key_numbers.png "key_numbers")

ジョイスティックのキー番号はトピック`/joy`で確認できます。

```sh
# ノードを起動する
roslaunch crane_x7_examples joystick_example.launch sim:=true

# 別のターミナルでコマンドを入力
rostopic echo /joy

# ジョイスティックのボタンを押す
header: 
  seq: 1
  stamp: 
    secs: 1549359364
    nsecs: 214800952
  frame_id: ''
axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
---
```

### obstacle_avoidance_example.pyの実行

ROSのServiceを使って、障害物の追加と障害物回避をするコード例です。

次のコマンドでノードを起動します。

```sh
roslaunch crane_x7_examples obstacle_avoidance_example.launch
```
