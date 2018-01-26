# CRANE-X7 ROSパッケージ beta

CRANE-X7の開発用ROSパッケージの置き場です．  
ベータ版の間はここで開発を行います．  

## crane_x7_ros

CRANE-X7の各パッケージはcrane_x7_rosにまとめています．   
GitHubからのcloneしやすさを意識したが、ひょっとすると正式リリースまでに構成が変わるかもしれません．  

製品ページはこちら．   
<http://www.rt-net.jp/crane-x7/>   

### crane_x7_description

CRANE-X7のモデルデータやリンクとジョイントの構成を定義するパッケージ．   
MoveItやGazeboから呼ばれる．   

[TODO]URDFをベタ書きしているのでXACRO化する予定あり

### crane_x7_control

CRANE-X7の制御を行うパッケージ．   
dynamixel_sdkのC++ライブラリが必要です．   
実機との通信に以下の操作が必要です．   

`sudo chmod 666 /dev/ttyUSB0`   

通信に使用するポートの名前やサーボ情報はconfig/crane_x7_control.yamlに記載します．   
設定されたUSBポートが無い場合、コントローラからの指示通りの値を返すダミージョイントモードで動作します．   
ハードウェアを使用しなくてもデバッグが出来るので便利に使って下さい．   

起動時は設定されたホームポジションへ5秒かけて移動します．   
ノードを停止するとサーボをブレーキモードに変更してから終了するので安全に停止することが出来ます．   

### crane_x7_moveit_config

MoveIt!のパッケージです．下記のコマンドで起動します．   

`roslaunch crane_x7_moveit_config demo.launch`   
