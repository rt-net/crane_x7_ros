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

ケーブルの接続ポート名はデフォルトで`/dev/ttyUSB0`です。
別のポート名(例: /dev/ttyUSB1)を使う場合は次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false port:=/dev/ttyUSB1
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
