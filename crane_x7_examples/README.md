## domino2.pyについての説明 

ドミノの高さを検出し、その高さに対応してドミノを立て、倒すというプログラムです。
なお、実機のみの対応となります。

## インストール


### REALSENSEの準備

REALSENSEをPCと接続し、以下のコマンドを実行します。

```
roslaunch realsense2_camera rs_camera.launch 
```

次のコマンドを実行することで、計算後のドミノ高さのTopicが送られるようになります。

```
cd catkin_ws/src/realsense-ros/realsense2_camera/scris
python show_center_depth_pub.py
```

### 実機の準備

実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
sudo chmod 777 /dev/ttyUSB0 
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

## 実行方法
