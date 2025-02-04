^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package crane_x7_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Prepare for release 4.4.0 (`#194 <https://github.com/rt-net/crane_x7_ros/issues/194>`_)
  4.4.0
* バージョンタグを4.3.0に更新 (`#184 <https://github.com/rt-net/crane_x7_ros/issues/184>`_)
* バージョンタグを4.2.0に更新 (`#177 <https://github.com/rt-net/crane_x7_ros/issues/177>`_)
  v4.2.0に更新
* Contributors: Kuwamai

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
* Contributors: Kuwamai

3.0.0 (2022-09-06)
------------------
* Merge pull request `#157 <https://github.com/rt-net/crane_x7_ros/issues/157>`_ from rt-net/ros2-devel
  Support ROS 2 Foxy
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
* launch時のwarning消すための修正 (`#149 <https://github.com/rt-net/crane_x7_ros/issues/149>`_)
  * crane_x7.srdfのtable virtual jointはスキップされているため削除する
  * kinematicsプラグインのattemptsオプションはMelodicから廃止されているので、削除する
  * default_planning_pipelineパラメータの設定と、planning_pipelineパラメータのネームスペースの変更
  * Revert "default_planning_pipelineパラメータの設定と、planning_pipelineパラメータのネームスペースの変更"
  This reverts commit 2d99307f9f02a316acabb2b9a0c75e8bdcdba560.
* Add default scaling factors (`#134 <https://github.com/rt-net/crane_x7_ros/issues/134>`_)
  * Add default scaling factors
  * Fix joint_limits.yaml for RViz scaling factor
  * add acceleration_scaling_factor for examples
* Remove deprecated param 'use_gui' (`#115 <https://github.com/rt-net/crane_x7_ros/issues/115>`_)
  * Remove deprecated param 'use_gui'
  * Update package.xml
* Fix conflicts
* Merge pull request `#72 <https://github.com/rt-net/crane_x7_ros/issues/72>`_ from rt-net/port_name\_`#71 <https://github.com/rt-net/crane_x7_ros/issues/71>`_
  Set dynamixel port name via launch arg. Close `#71 <https://github.com/rt-net/crane_x7_ros/issues/71>`_
* Fix conflicts
* Merge pull request `#85 <https://github.com/rt-net/crane_x7_ros/issues/85>`_ from rt-net/gazebo
  Add crane_x7_gazebo
* Can control crane_x7 in gazebo with MoveIt.
* Set dynamixel port name via launch arg
* Update package.xml
* Merge pull request `#48 <https://github.com/rt-net/crane_x7_ros/issues/48>`_ from rt-net/`#47 <https://github.com/rt-net/crane_x7_ros/issues/47>`__modify_moveit_run_depend
  Add run_depend(moveit_simple_controller_manager).
  close `#47 <https://github.com/rt-net/crane_x7_ros/issues/47>`_
* Add run_depend(moveit_simple_controller_manager).
* Merge pull request `#46 <https://github.com/rt-net/crane_x7_ros/issues/46>`_ from rt-net/`#45 <https://github.com/rt-net/crane_x7_ros/issues/45>`__modify_max_acceleration
  Modify moveit max_accelerations.
  close `#45 <https://github.com/rt-net/crane_x7_ros/issues/45>`_
* Modify moveit max_accelerations.
* Merge pull request `#43 <https://github.com/rt-net/crane_x7_ros/issues/43>`_ from rt-net/`#41 <https://github.com/rt-net/crane_x7_ros/issues/41>`__modify_finger_collision
  `#41 <https://github.com/rt-net/crane_x7_ros/issues/41>`_ modify finger collision
  close `#43 <https://github.com/rt-net/crane_x7_ros/issues/43>`_
* Modify finger collision.
* Merge pull request `#40 <https://github.com/rt-net/crane_x7_ros/issues/40>`_ from rt-net/`#28 <https://github.com/rt-net/crane_x7_ros/issues/28>`__add_virtual_joint
  `#28 <https://github.com/rt-net/crane_x7_ros/issues/28>`_ add virtual joint
  close `#28 <https://github.com/rt-net/crane_x7_ros/issues/28>`_
* Modify author name.
* Add virtual joint(table )。
* Merge pull request `#31 <https://github.com/rt-net/crane_x7_ros/issues/31>`_ from rt-net/`#30 <https://github.com/rt-net/crane_x7_ros/issues/30>`__moveit_fake_launch
  `#30 <https://github.com/rt-net/crane_x7_ros/issues/30>`_ moveit fake launch
* Modify trajectory constraints
* Modify collision setting
* Modify node parameters
* Revert "Modify node parameters"
  This reverts commit f105ca10be0bc9411a96cfa0bf52466db48f628f.
* Revert "Modify collision setting"
  This reverts commit 7a2278cb0e0ffb3dc3efda9a36a0d96d37135a24.
* Revert "Modify trajectory constraints"
  This reverts commit 25525c8d61917f7d41729d60b8162c7393dda4eb.
* Modify trajectory constraints
* Modify collision setting
* Modify node parameters
* Merge pull request `#23 <https://github.com/rt-net/crane_x7_ros/issues/23>`_ from rt-net/`#22 <https://github.com/rt-net/crane_x7_ros/issues/22>`__update_license
  Update license and author
* Update license and author
* Merge branch 'master' into collision_meshes
* Merge pull request `#4 <https://github.com/rt-net/crane_x7_ros/issues/4>`_ from rt-net/urdf_refactor
  Urdf refactor
* Allow controlling fake_execution in the MoveIt demo
* Fix controller names and spaces
* Re-built moveit config for new URDF
* Merge branch 'master' into urdf_refactor
* Merge pull request `#3 <https://github.com/rt-net/crane_x7_ros/issues/3>`_ from rt-net/moveit_pose_modify
  Moveit pose modify
* moveit pose modify(old desigh)
* First commit.
* Contributors: Daisuke Sato, Geoffrey Biggs, RT Corp, Shota Aoki, ShotaAk, Tiryoh, nomumu, nomumuRT
