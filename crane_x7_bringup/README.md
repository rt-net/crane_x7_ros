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

[![Image from Gyazo](https://i.gyazo.com/213982c623f07cffefce73469d2e84b1.gif)](https://gyazo.com/213982c623f07cffefce73469d2e84b1)

### 実機を使う場合

CRANE-X7から20cm離れた位置にピッキング対象を設置します。

[![Image from Gyazo](https://i.gyazo.com/7484a1712a3454092e525c3bbe1a0b3c.jpg)](https://gyazo.com/7484a1712a3454092e525c3bbe1a0b3c)

サンプルで使用しているこのオレンジ色のソフトボールはRT ROBOT SHOPの[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)から入手することができます。

Terminalを開き、 `crane_x7_bringup` の `demo.launch` を起動します。

```
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

次に、もう1つTerminalを開き、 `crane_x7_bringup` の `crane_x7_pick_and_place_demo.py` を実行するとCRANE-X7にPick and Placeのデモを実行させることができます。

```
rosrun crane_x7_bringup crane_x7_pick_and_place_demo.py
```

[![Image from Gyazo](https://i.gyazo.com/b861f09744da8198066ca65c234043eb.gif)](https://gyazo.com/b861f09744da8198066ca65c234043eb)
