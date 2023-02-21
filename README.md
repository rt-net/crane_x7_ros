[English](README.en.md) | [日本語](README.md)

# crane_x7_ros

[![industrial_ci](https://github.com/rt-net/crane_x7_ros/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/crane_x7_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

![crane_x7_gazebo](https://rt-net.github.io/images/crane-x7/crane_x7_gazebo.png "crane_x7_gazebo")

CRANE-X7のROSパッケージです。

製品ページはこちらです。  
[https://www.rt-net.jp/products/crane-x7](https://www.rt-net.jp/products/crane-x7)

ROS Wikiはこちらです。  
[https://wiki.ros.org/crane_x7](https://wiki.ros.org/crane_x7)

ROSのサンプルコード集はこちらです。  
[crane_x7_examples](https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples)

## サポートするROSディストリビューション

- Melodic
- Noetic

### ROS 2

- [Foxy](https://github.com/rt-net/crane_x7_ros/tree/foxy-devel)
- [Humble](https://github.com/rt-net/crane_x7_ros/tree/humble-devel)

## インストール方法

### ソースからビルドする方法

- [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)を参照しROSをインストールします。

- `git`を使用して本パッケージをダウンロードします。

  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_ros.git
  ```

- [crane_x7_description](https://github.com/rt-net/crane_x7_description)パッケージをダウンロードします。
このパッケージには株式会社アールティの[非商用ライセンス](https://github.com/rt-net/crane_x7_description/blob/master/LICENSE)が適用されています。

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/crane_x7_description.git
  ```

- 依存関係にあるパッケージをインストールします。

  ```bash
  cd ~/catkin_ws/src
  rosdep install -r -y --from-paths . --ignore-src
  ```

- `catkin_make`を使用して本パッケージをビルドします。

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### v1.0.0以前のバージョンからv2.x.xへ更新する場合

バージョンの違いについては
https://github.com/rt-net/crane_x7_ros/issues/154
を参照してください。

次の手順でパッケージを更新してください。

```bash
# crane_x7_rosを更新
cd ~/catkin_ws/src/crane_x7_ros
git pull origin master

# crane_x7_descriptionをダウンロード
cd ~/catkin_ws/src
git clone https://github.com/rt-net/crane_x7_description.git
rosdep install -r -y --from-paths . --ignore-src

# ビルド環境を初期化し、パッケージを再ビルド
# 同じワークスペースにある、CRANE-X7以外の他のROSパッケージについても再ビルドを行います
cd ~/catkin_ws
rm -r build devel
catkin_make
```

## セットアップ方法

`crane_x7_control`が実機と通信する際には`/dev/ttyUSB0`へのアクセス権が必要です。
`/dev/ttyUSB0`へのアクセス権を変更するには下記のコマンドを実行します。

```bash
sudo chmod 666 /dev/ttyUSB0
```

## パッケージ概要

CRANE-X7の各パッケージはcrane_x7_rosにまとめています。  

### crane_x7_control

CRANE-X7の制御を行うパッケージです。  
dynamixel_sdkのC++ライブラリが必要です。  
実機との通信には`/dev/ttyUSB0`へのアクセス権が必要です。

通信に使用するポートの名前やサーボ情報は`config/crane_x7_control.yaml`に記載します。  
設定されたUSBポートが無い場合、コントローラからの指示通りの値を返すダミージョイントモードで動作します。  
ハードウェアを使用しなくてもデバッグが出来るので便利に使って下さい。  

起動時は設定されたホームポジションへ5秒かけて移動します。  
ノードを停止するとサーボをブレーキモードに変更してから終了するので安全に停止することができます。  

### crane_x7_moveit_config

MoveItのパッケージです。下記のコマンドで起動します。

`roslaunch crane_x7_moveit_config demo.launch`

### crane_x7_bringup

CRANE-X7の起動に必要なlaunchファイルをまとめたパッケージです。

### crane_x7_examples

サンプルコード集です。
使い方については[./crane_x7_examples/README.md](./crane_x7_examples/README.md)を参照してください。

### crane_x7_gazebo

GazeboでCRANE-X7のシミュレーションを行うパッケージです。

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

`roslaunch crane_x7_gazebo crane_x7_with_table.launch`

---

## ライセンス

(C) 2018 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[crane_x7_description](https://github.com/rt-net/crane_x7_description)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[crane_x7_description/LICENSE](https://github.com/rt-net/crane_x7_description/blob/master/LICENSE)を参照してください。

## 開発について

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](./CONTRIBUTING.md)に従ってください。
