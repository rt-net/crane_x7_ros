# crane_x7_bringup

CRANE-X7のためのパッケージ、 `crane_x7_ros` のbringupパッケージです。

## 使い方

### シミュレータを使う場合

Terminalを開き、 `crane_x7_bringup` の `demo.launch` を起動します。

```
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

次に、もう1つTerminalを開き、 `crane_x7_bringup` の `crane_x7_pick_and_place_demo.py` を実行するとCRANE-X7にPick and Placeのデモを実行させることができます。

```
rosrun crane_x7_bringup crane_x7_pick_and_place_demo.py
```

![bringup_rviz](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup_rviz.gif "bringup_rviz")

### 実機を使う場合

CRANE-X7から20cm離れた位置にピッキング対象を設置します。

![bringup](https://github.com/rt-net/crane_x7_ros/blob/images/images/bringup.jpg "bringup")

サンプルで使用しているこのオレンジ色のソフトボールはRT ROBOT SHOPの[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)から入手することができます。

Terminalを開き、 `crane_x7_bringup` の `demo.launch` を起動します。

```
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

次に、もう1つTerminalを開き、 `crane_x7_bringup` の `crane_x7_pick_and_place_demo.py` を実行するとCRANE-X7にPick and Placeのデモを実行させることができます。

```
rosrun crane_x7_bringup crane_x7_pick_and_place_demo.py
```

デモ動画は[こちら](https://youtu.be/_8xBgpgMhk8)。
