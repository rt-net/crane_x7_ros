# crane_x7_ros

CRANE-X7のROSパッケージです．  

製品ページはこちらです．  
[https://www.rt-net.jp/crane-x7/](https://www.rt-net.jp/crane-x7/)

ROS Wikiはこちらです．  
[https://wiki.ros.org/crane_x7](https://wiki.ros.org/crane_x7)

## 動作環境

以下の環境にて動作確認を行っています．

- OS
  - Ubuntu 16.04.5 LTS
- ROS
  - ROS Kinetic Kame 1.12.14
  - Rviz 1.12.16
  - MoveIt! 0.9.12

## インストール方法

### ソースからビルドする方法

- [ROS Wiki](http://wiki.ros.org/ja/kinetic/Installation/Ubuntu)を参照しROSをインストールします．

- `git`を使用して本パッケージをダウンロードします．

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_ros.git
  ```

- 依存関係にあるパッケージをインストールします．

  ```bash
  cd ~/catkin_ws/src
  rosdep install -r -y --from-paths crane_x7_ros
  ```

- `catkin_make`を使用して本パッケージをビルドします．

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### `apt`を使用してインストールする方法

後日提供予定です．

## セットアップ方法

`crane_x7_control`が実機と通信する際には`/dev/ttyUSB0`へのアクセス権が必要です．
`/dev/ttyUSB0`へのアクセス権を変更するには下記のコマンドを実行します．

```bash
sudo chmod 666 /dev/ttyUSB0
```

## パッケージ概要

CRANE-X7の各パッケージはcrane_x7_rosにまとめています．  

### crane_x7_description

CRANE-X7のモデルデータやリンクとジョイントの構成を定義するパッケージです．  
MoveIt!やGazeboから呼び出されます．

### crane_x7_control

CRANE-X7の制御を行うパッケージです．  
dynamixel_sdkのC++ライブラリが必要です．  
実機との通信には`/dev/ttyUSB0`へのアクセス権が必要です．

通信に使用するポートの名前やサーボ情報は`config/crane_x7_control.yaml`に記載します．  
設定されたUSBポートが無い場合、コントローラからの指示通りの値を返すダミージョイントモードで動作します．  
ハードウェアを使用しなくてもデバッグが出来るので便利に使って下さい．  

起動時は設定されたホームポジションへ5秒かけて移動します．  
ノードを停止するとサーボをブレーキモードに変更してから終了するので安全に停止することができます．  

### crane_x7_moveit_config

MoveIt!のパッケージです．下記のコマンドで起動します．  

`roslaunch crane_x7_moveit_config demo.launch`

### crane_x7_bringup

初めて`crane_x7_ros`を使うことを意識して作成されたbringupパッケージです．

`crane_x7_bringup` の使い方については[./crane_x7_bringup/README.md](./crane_x7_bringup/README.md)を参照してください．
