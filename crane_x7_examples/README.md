### domino2.pyについての説明 

ドミノの高さを検出し、その高さに対応してドミノを立て、倒すというプログラムです。
なお、実機のみの対応となります。

## 使用するドミノ

縦8mm、横5mm、厚さ1.6mmのドミノをスタイロフォームを使って四枚自作しました。
違う大きさのドミノを使う場合は適宜プログラムの値を変更する必要があります(グリッパーの値やheight、width等)。

## インストール
本プログラムのインストール

```
git clone 
```

realsense-rosのインストールは[こちらのサイト](https://demura.net/misc/14263.html)の手順に従いました。

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
送られているかどうかは、次のコマンドで確認できます。
```
rostopic echo /depth
```

### 実機の準備

制御信号ケーブルを接続した状態で次のコマンドを実行します。エラーが出た場合は挿し直してもう一度実行してください。
```sh
sudo chmod 777 /dev/ttyUSB
```
次のコマンドを実行すると、crane_x7がゆっくり直立します。
```
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

## 実行方法
次のコマンドを実行することで動きます。
```
rosrun crane_x7_examples domino2.py 
```
## 成功率について
今回使用したドミノでは、４つ全て立つ確率は三割ほどでした。成功率を上げるにはドミノを大きくする(太くする)、またそれに応じたプログラムの調整が必要になります。
