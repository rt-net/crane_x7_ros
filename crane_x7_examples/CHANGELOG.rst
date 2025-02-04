^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package crane_x7_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Mock components対応 (`#201 <https://github.com/rt-net/crane_x7_ros/issues/201>`_)
  mock componentsオプションを追加
* Jazzy対応 (`#198 <https://github.com/rt-net/crane_x7_ros/issues/198>`_)
  * crane_plushのjazzy対応を参考に設定ファイルをjazzy対応
  * .h->.hppに対応
  * idustrial_ci.yamlをjazzyに対応
  * crane_x7_moveit_configのPackage.xmlにauthor追加
  * ファイル名変更controlles.yaml->moveit_controlles.yaml
  * Setup Assistantで生成した設定ファイルを追加
  * crane_x7_descriptionのjazzyブランチに対応させる
  * Update README.md
  * Update README.en.md
  * MoveItConfigsBuilderによるMoveit設定に対応
  * import yamlを復活
  * gz_sim対応
  * ブランチ名修正
  * Update README
  * Update README
  * use_d435によるrvizコンフィグファイルパスの切り替え
  * 変数名、パラメータ変更
  * RVizコンフィグファイル変更
  * 不要な記述を削除
  * setup_assistant設定追加
  * パッケージ情報更新
  * スタイル修正
  * Update README
  * Gazeboカメラ位置調整
  * move_group_capabilitiesの変更に対応
  * 依存パッケージ修正
  * CIのバージョン更新
  Co-authored-by: YusukeKato <YusukeKato@users.noreply.github.com>
  * ヘッダーファイル名修正
  * computeCartesianPathの変更に対応
  ---------
  Co-authored-by: mizonon <mizoguchi@rt-net.jp>
  Co-authored-by: YusukeKato <YusukeKato@users.noreply.github.com>
* RealSenseのcamera_namespace対応 (`#197 <https://github.com/rt-net/crane_x7_ros/issues/197>`_)
* Prepare for release 4.4.0 (`#194 <https://github.com/rt-net/crane_x7_ros/issues/194>`_)
  4.4.0
* goal_toleranceの設定と把持角の調整 (`#193 <https://github.com/rt-net/crane_x7_ros/issues/193>`_)
  * goal_tolerance調整
  * グリッパ角調整
* バージョンタグを4.3.0に更新 (`#184 <https://github.com/rt-net/crane_x7_ros/issues/184>`_)
* 動画URL追加
* 色認識サンプルの追加 (`#183 <https://github.com/rt-net/crane_x7_ros/issues/183>`_)
  * 色認識追加
  * 二値化画像を配信
  * 不要な処理を削除
  * RVizに二値化画像を表示
  * 距離のオフセット追加
  * 変数名変更
  * カーネルを変更
  * 不要な空行を削除
  * 画像取得時のheaderをtfに使用
  * コメント、変数名修正
  * Update README
  * コメント修正
  * 変数名変更
* バージョンタグを4.2.0に更新 (`#177 <https://github.com/rt-net/crane_x7_ros/issues/177>`_)
  v4.2.0に更新
* 点群認識サンプルの追加 (`#176 <https://github.com/rt-net/crane_x7_ros/issues/176>`_)
  * 点群を0.5mの距離でフィルタリング
  * Voxel gridでダウンサンプリング
  * 点群の座標変換
  * 点群の座標変換修正
  * 点群の取得範囲変更
  * KdTreeによるクラスタリング
  * 認識した物体位置をtfで配信
  * スタイル修正
  * ファイル名変更
  * point_cloud_detectionをサンプル実行用launchに追加
  * 不要な変数を削除
  * Update README
  * プレース後の物体を再度把持してしまうためプレース位置修正
  * 認識後の点群をRvizに表示
  * 平面検出処理追加
  * コメント修正
  * READMEに動画URL追加
  * tfの表示時間短縮
  * タイムスタンプを一致させるためheaderをコピー
  * 点群の重心位置を物体位置として配信
  * tfの取得時間の範囲を変更
  * 物体位置の高さ制限を追加
  * 平面検出をコメントアウト
  * newをmake_sharedに置き換え
  * 点群処理を複数の関数に分割
  * Copyright表記修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * Update README
  * 変数の型修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 変数の型修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 変数名修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 点群取得範囲修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 平面検出の判定条件修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * for文を範囲for文に変更
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * for文を範囲for文に変更
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 認識範囲に点群がない場合INFOを出力
  ---------
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* Contributors: Kuwagata, Kuwamai

4.3.0 (2023-06-14)
------------------
* ros2ブランチの更新内容を反映 (`#188 <https://github.com/rt-net/crane_x7_ros/issues/188>`_)
  * 色認識サンプルの追加 (`#183 <https://github.com/rt-net/crane_x7_ros/issues/183>`_)
  * 色認識追加
  * 二値化画像を配信
  * 不要な処理を削除
  * RVizに二値化画像を表示
  * 距離のオフセット追加
  * 変数名変更
  * カーネルを変更
  * 不要な空行を削除
  * 画像取得時のheaderをtfに使用
  * コメント、変数名修正
  * Update README
  * コメント修正
  * 変数名変更
  * 動画URL追加
  * バージョンタグを4.3.0に更新 (`#184 <https://github.com/rt-net/crane_x7_ros/issues/184>`_)
* Contributors: Kuwamai

4.2.0 (2023-02-21)
------------------
* Merge pull request `#178 <https://github.com/rt-net/crane_x7_ros/issues/178>`_ from rt-net/add_pointcloud_detection_for_humble
  ros2ブランチの更新内容を反映
* バージョンタグを4.2.0に更新 (`#177 <https://github.com/rt-net/crane_x7_ros/issues/177>`_)
  v4.2.0に更新
* 点群認識サンプルの追加 (`#176 <https://github.com/rt-net/crane_x7_ros/issues/176>`_)
  * 点群を0.5mの距離でフィルタリング
  * Voxel gridでダウンサンプリング
  * 点群の座標変換
  * 点群の座標変換修正
  * 点群の取得範囲変更
  * KdTreeによるクラスタリング
  * 認識した物体位置をtfで配信
  * スタイル修正
  * ファイル名変更
  * point_cloud_detectionをサンプル実行用launchに追加
  * 不要な変数を削除
  * Update README
  * プレース後の物体を再度把持してしまうためプレース位置修正
  * 認識後の点群をRvizに表示
  * 平面検出処理追加
  * コメント修正
  * READMEに動画URL追加
  * tfの表示時間短縮
  * タイムスタンプを一致させるためheaderをコピー
  * 点群の重心位置を物体位置として配信
  * tfの取得時間の範囲を変更
  * 物体位置の高さ制限を追加
  * 平面検出をコメントアウト
  * newをmake_sharedに置き換え
  * 点群処理を複数の関数に分割
  * Copyright表記修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * Update README
  * 変数の型修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 変数の型修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 変数名修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 点群取得範囲修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 平面検出の判定条件修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * for文を範囲for文に変更
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * for文を範囲for文に変更
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * 認識範囲に点群がない場合INFOを出力
  ---------
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* Contributors: Kuwamai, Shota Aoki

4.1.0 (2023-01-30)
------------------
* バージョン表記の更新 (`#174 <https://github.com/rt-net/crane_x7_ros/issues/174>`_)
* ArUcoサンプルとピッキング動作の追加  (`#173 <https://github.com/rt-net/crane_x7_ros/issues/173>`_)
  * ArUcoサンプルとピッキング動作の追加 (`#169 <https://github.com/rt-net/crane_x7_ros/issues/169>`_)
  * arucoマーカの認識
  * マーカ位置をm系に変換
  * マーカ位置姿勢をtfに配信
  * tfのsubscribe
  * マーカ位置が静止していることを検知
  * スタイル修正
  * picking動作追加
  * 不要な記述を削除
  * スタイル修正
  * 不要な行を削除
  * マーカ位置修正
  * 把持姿勢の調整
  * tf確認用rviz config追加
  * URL更新
  * マーカ認識位置のオフセットを削除
  * Rviz configの切り替えを実装
  * デフォルトで起動するスクリプトを修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * ArUcoの辞書についてコメント追加
  * 変数名修正
  * デバッグ用処理を削除
  * 変数名変更
  * コメント追加
  * tfの時間計算を修正
  * 現在時刻の取得方法変更
  * 不要なモジュールの削除
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * Update README
  * Update README
  * PythonExpressionをUnlessConditionに置き換え
  * 複数マーカに対応
  (cherry picked from commit aa3792b54ae873125a34611f93a0fecec57466c6)
  * 把持対象のframe id変更
  * スタイル修正
  * 待機姿勢を関節負荷の低いものに変更
  * Update README
  * Update README
  * スタイル修正
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * サンプル動画URL更新 (`#172 <https://github.com/rt-net/crane_x7_ros/issues/172>`_)
  * READMEに動画追加
  * README修正
  * tf2_geometry_msgs.hppの更新
  * gz_ros2_controlのブランチを変更
  * コメント修正
  * コメント修正
  * Update README
  ---------
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* Humble版パッケージのバージョン表記更新 (`#166 <https://github.com/rt-net/crane_x7_ros/issues/166>`_)
  バージョン表記の更新
* Merge pull request `#165 <https://github.com/rt-net/crane_x7_ros/issues/165>`_ from rt-net/fix_examples_sim_time
  Add use_sim_time arg to example.launch.py
* Add use_sim_time option to example.launch.py
* Support ROS 2 Humble (`#162 <https://github.com/rt-net/crane_x7_ros/issues/162>`_)
  * gazeboのコンフィグファイルをgazebo6のデフォルトファイルから移植
  * gazebo動作確認のため一時的にコントローラyamlを移植
  joint_state_broadcasterに変更
  全ノードにuse_sim_timeを設定
  * joint_state_broadcasterの更新
  * controller_managerのoutputとspawnerを更新
  * HardwareInterfaceのHumble対応
  * CIのビルド環境修正
  * .ci.rosinstallにgz_ros2_control追加
  * ros_ign_gazeboをros_gz_simに更新
  * joint_valuesの駆動速度を下げた
  * スタイル修正
  * Update README
  * Update README
  * Update README
  * Update README.en.md
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * Update README.md
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  * max_velocityを設定
  * 不要になったファイルを削除
  * 依存パッケージを更新
  Co-authored-by: ShotaAk <s.aoki@rt-net.jp>
* RealSense D435搭載モデルに対応 (`#164 <https://github.com/rt-net/crane_x7_ros/issues/164>`_)
  * use_d435引数を追加
  * 引数use_d435を追加
  * スタイル修正
  * スタイル修正
  * 依存パッケージにrealsense2_cameraを追加
  * Update README
  * Gazebo上でuse_d435の使用停止
  * Update crane_x7_examples/README.md
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* Contributors: Kuwamai, ShotaAk

3.0.0 (2022-09-06)
------------------
* サンプル実行時の初期姿勢を追加 (`#159 <https://github.com/rt-net/crane_x7_ros/issues/159>`_)
* Merge pull request `#157 <https://github.com/rt-net/crane_x7_ros/issues/157>`_ from rt-net/ros2-devel
  Support ROS 2 Foxy
* README更新
* バージョンをv3.0.0に更新
* Support ROS 2
* Contributors: Kuwamai, ShotaAk

2.0.0 (2022-05-27)
------------------
* Apache-2.0ライセンスを適用。バージョンを2.0.0に更新 (`#156 <https://github.com/rt-net/crane_x7_ros/issues/156>`_)
  * ライセンスをApache-2.0へ変更
  * package.xmlのバージョンとライセンスを変更
  * cartesian_path_exampleのライセンス変更
  * ライセンスに関わる文言を変更。開発について　の項目を追加
  * CONTRIBUTING.mdを追加
  * Pythonスクリプトにライセンスを明記
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Contributors: Shota Aoki

1.0.0 (2022-05-12)
------------------
* package.xmlのversionタグを1.0.0に更新。authorを修正。 (`#153 <https://github.com/rt-net/crane_x7_ros/issues/153>`_)
  * package.xmlのversionタグを1.0.0に変更
  * authorを修正
* MoveIt!をMoveItに修正 (`#145 <https://github.com/rt-net/crane_x7_ros/issues/145>`_)
  * industrial_ciからkinetic環境を削除
  * READMEからKineticサポートの文章を削除
  * MoveItのパッケージ名を修正
* Fix CMakeLists.txt (`#139 <https://github.com/rt-net/crane_x7_ros/issues/139>`_)
* Add default scaling factors (`#134 <https://github.com/rt-net/crane_x7_ros/issues/134>`_)
  * Add default scaling factors
  * Fix joint_limits.yaml for RViz scaling factor
  * add acceleration_scaling_factor for examples
* Support ROS Noetic (`#130 <https://github.com/rt-net/crane_x7_ros/issues/130>`_)
  * Add xacro prefix to macro tags
  * Fix example fortmats
  * update crane_x7_control CMakeLists.txt
  * update exmaple code formats
  * Update CI
* Fix the preset_pid_gain_example to be safe motion. (`#123 <https://github.com/rt-net/crane_x7_ros/issues/123>`_)
  * Fix prest_pid_gain_example
  * Fix a print text
* Add a cartesian_path_example (`#121 <https://github.com/rt-net/crane_x7_ros/issues/121>`_)
  * Add cartesian_path_example
  * Update README
  * Add comment and rename a function
* Port images from the images branch to rt-net.github.io/images (`#120 <https://github.com/rt-net/crane_x7_ros/issues/120>`_)
  * Update README.md
  * Update README.en.md
  * Update README.md
  * Update README.md
  * Update README.md
  * Update README.en.md
* Merge pull request `#106 <https://github.com/rt-net/crane_x7_ros/issues/106>`_ from rt-net/fix_readme
  トップページのREADMEの修正
* Fix crane_x7_examples/READMEs
* Merge branch 'master' of https://github.com/rt-net/crane_x7_ros
* Update README.md
* Update README.md
* Merge pull request `#105 <https://github.com/rt-net/crane_x7_ros/issues/105>`_ from rt-net/translation
  Tranlate Japanese to English
* Update README.md
* Update README.md
* Fix typo
* Update examples/README.md
* Translate crane_x7_examples/README.md
* Merge pull request `#104 <https://github.com/rt-net/crane_x7_ros/issues/104>`_ from rt-net/add_new_sample
  Add new sample
* Merge branch 'add_new_sample' of github.com:rt-net/crane_x7_ros into add_new_sample
* Update joint_values_example
* Update README.md
* Add a new sample joint_values_example.py
* Merge pull request `#99 <https://github.com/rt-net/crane_x7_ros/issues/99>`_ from rt-net/add_gif_images
  Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#87 <https://github.com/rt-net/crane_x7_ros/issues/87>`_ from rt-net/gazebo_with_pick_and_place
  Add pick and place example for crane_x7_gazebo. Close `#83 <https://github.com/rt-net/crane_x7_ros/issues/83>`_
* Update README.md
* Update README.md
* Add pick and place example for gazebo
* Fix conflicts
* Merge pull request `#72 <https://github.com/rt-net/crane_x7_ros/issues/72>`_ from rt-net/port_name\_`#71 <https://github.com/rt-net/crane_x7_ros/issues/71>`_
  Set dynamixel port name via launch arg. Close `#71 <https://github.com/rt-net/crane_x7_ros/issues/71>`_
* Fix conflicts
* Merge pull request `#85 <https://github.com/rt-net/crane_x7_ros/issues/85>`_ from rt-net/gazebo
  Add crane_x7_gazebo
* Can control crane_x7 in gazebo with MoveIt.
* Merge pull request `#79 <https://github.com/rt-net/crane_x7_ros/issues/79>`_ from rt-net/servo_info\_`#69 <https://github.com/rt-net/crane_x7_ros/issues/69>`_
  Add Servo info example. Close `#69 <https://github.com/rt-net/crane_x7_ros/issues/69>`_
* Update README.md
* Merge branch 'servo_info\_`#69 <https://github.com/rt-net/crane_x7_ros/issues/69>`_' of github.com:rt-net/crane_x7_ros into servo_info\_`#69 <https://github.com/rt-net/crane_x7_ros/issues/69>`_
* Add units to print message
* Fix typo
* Update README.md
* Merge remote-tracking branch 'origin/master' into servo_info\_`#69 <https://github.com/rt-net/crane_x7_ros/issues/69>`_
* Merge pull request `#78 <https://github.com/rt-net/crane_x7_ros/issues/78>`_ from rt-net/obstacle\_`#68 <https://github.com/rt-net/crane_x7_ros/issues/68>`_
  Add obstacle avoidance example. Close `#68 <https://github.com/rt-net/crane_x7_ros/issues/68>`_
* Update README.md
* Update README.md
* Add floor object
* Server shutdown with rospy shutdown
* Fix conflict
* Add servo_info_example
* Change max velocity 0.3 to 0.1
* Update README.md
* Add sleep_time for scene update
* Obstacle avoidance example uses Service
* Merge pull request `#77 <https://github.com/rt-net/crane_x7_ros/issues/77>`_ from rt-net/hotfix_joy_preset
  Fix preset No. in joystick_example
* Update README.md
* Fix preset No. in joystick_example
* Add obstacle aoivdance example
* Merge pull request `#76 <https://github.com/rt-net/crane_x7_ros/issues/76>`_ from rt-net/hotfix_joy
  Hotfix joystick example
* Merge pull request `#75 <https://github.com/rt-net/crane_x7_ros/issues/75>`_ from rt-net/teaching_sample
  Add Teaching sample
* Fix an exception of set_joint_value_target
* Update README.md
* Add exception of set joint value target
* Add exception of setting joint target
* Add Loop play to teaching_example
* Fix message
* Update README.md
* Update README.md
* Update README.md
* Fix control commands of teaching_example
* Update README.md
* Add teaching_example
* Merge pull request `#74 <https://github.com/rt-net/crane_x7_ros/issues/74>`_ from rt-net/joy\_`#67 <https://github.com/rt-net/crane_x7_ros/issues/67>`_
  Add joystick example. close `#67 <https://github.com/rt-net/crane_x7_ros/issues/67>`_
* Remove sleep function at wakeup
* Update README.md
* Add run depend to package.xml
* Update README.md
* Rename KeyConfig
* Add teaching mode for joystick_example
* Add sim launch for joystick_example
* Add PID preset function to joystick_example
* Joystick example controls arm pose and gripper angle
* Add joystick_example
* Set dynamixel port name via launch arg
* Merge pull request `#65 <https://github.com/rt-net/crane_x7_ros/issues/65>`_ from rt-net/`#64 <https://github.com/rt-net/crane_x7_ros/issues/64>`__pid_gain_preset
  Add pid gain preset example. close `#64 <https://github.com/rt-net/crane_x7_ros/issues/64>`_
* Update README, examples
* Add preset_pid_gain_example
* Update README.md
* Update README.md
* Merge pull request `#61 <https://github.com/rt-net/crane_x7_ros/issues/61>`_ from rt-net/`#60 <https://github.com/rt-net/crane_x7_ros/issues/60>`__separate_package
  Separate examples from bringup package
* Separate examples from bringup package
* Contributors: Daisuke Sato, Shota Aoki, ShotaAk, Tiryoh
