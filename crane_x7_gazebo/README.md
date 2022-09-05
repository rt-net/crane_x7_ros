# crane_x7_gazebo

CRANE-X7 の[Gazebo](https://gazebosim.org/home)
シミュレーションパッケージです。

## ノードの起動

次のコマンドを実行するとGazeboが起動し、CRANE-X7モデルとTable、Boxが表示されます。

初回起動時はTableとBoxのモデルをダウンロードするため、モデルの表示に時間がかかることがあります。

実機との接続や`crane_x7_examples/launch/demo.launch/py`の実行は必要ありません。

```sh
$ ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

![crane_x7_gazebo](https://rt-net.github.io/images/crane-x7/crane_x7_gazebo_ros2.png)
