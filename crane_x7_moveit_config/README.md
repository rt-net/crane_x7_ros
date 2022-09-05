# crane_x7_moveit_config

このパッケージはCRANE-X7のmove_group設定ファイル及びlaunchファイルを含んでいます。

## ノードの起動

`run_move_group.launch.py`を実行すると、`move_group`や`rviz`等のノードが起動します。
コントローラノードは起動しないため、
CRANE-X7本体を動かすことはできません。(`crane_x7_examples`を参照してください。)

## configファイル

- controllers.yaml
  - `moveit_simple_controller_manager`のパラメータを設定しています
  - 設定内容は`crane_x7_control`のコントローラ名やコントローラタイプに依存します
- crane_x7.srdf
  - move_groupとして`arm`、`gripper`を設定しています
  - `arm`のgourp_stateとして`home`、`vertical`を設定しています
- kinematics.yaml
  - `arm`のkinematics_solverを設定しています
  - デフォルトの`KDLKinematicsPlugin`を使用しています
- ompl_planning.yaml
  - Open Motion Planning Libraryのパラメータを設定しています
  - [ros-planning/moveit_resources/panda_moveit_config](https://github.com/ros-planning/moveit_resources/tree/master/panda_moveit_config)のパラメータを流用しています
- joint_limits.yaml
  - ジョイントの速度、加速度リミットを設定しています
  - 速度リミットはURDFで定義したパラメータが参照されます
