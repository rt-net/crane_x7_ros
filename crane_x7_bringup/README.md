# crane_x7_bringup

CRANE X7のためのパッケージ、 `crane_x7_ros` のbringupパッケージです。

## 使い方

### シミュレータを使う場合

Terminalを開き、 `crane_x7_bringup` の `demo.launch` を起動します。

```
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

次に、もう1つTerminalを開き、 `crane_x7_bringup` の `crane_x7_pick_and_place_demo.py` を実行するとCRANE X7にPick and Placeのデモを実行させることができます。
```
rosrun crane_x7_bringup crane_x7_pick_and_place_demo.py
```

### 実機を使う場合

CRANE X7から20cm離れた位置にピッキング対象を設置します。

[![Image from Gyazo](https://i.gyazo.com/3423f79c9edecbf435df3505863ed181.png)](https://gyazo.com/3423f79c9edecbf435df3505863ed181)

Terminalを開き、 `crane_x7_bringup` の `demo.launch` を起動します。

```
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

次に、もう1つTerminalを開き、 `crane_x7_bringup` の `crane_x7_pick_and_place_demo.py` を実行するとCRANE X7にPick and Placeのデモを実行させることができます。
```
rosrun crane_x7_bringup crane_x7_pick_and_place_demo.py
```