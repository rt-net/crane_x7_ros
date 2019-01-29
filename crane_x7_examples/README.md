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


### crane_x7_pick_and_place_demo.pyの実行

モノを掴む・持ち上げる・運ぶ・置くコード例です。
次のコマンドを実行します。

```
rosrun crane_x7_examples crane_x7_pick_and_place_demo.py
```

![bringup_rviz](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup_rviz.gif "bringup_rviz")

**実機を使う場合**

CRANE-X7から20cm離れた位置にピッキング対象を設置します。

![bringup](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup.jpg "bringup")

サンプルで使用しているこのオレンジ色のソフトボールはRT ROBOT SHOPの[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)から入手することができます。

動作させると[こちら](https://youtu.be/_8xBgpgMhk8)のような動きになります。
