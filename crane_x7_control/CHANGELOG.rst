^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package crane_x7_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Prepare for release 4.4.0 (`#194 <https://github.com/rt-net/crane_x7_ros/issues/194>`_)
  4.4.0
* goal_toleranceの設定と把持角の調整 (`#193 <https://github.com/rt-net/crane_x7_ros/issues/193>`_)
  * goal_tolerance調整
  * グリッパ角調整
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
* Remove definition of controller that has already been loaded (`#151 <https://github.com/rt-net/crane_x7_ros/issues/151>`_)
* パラメータ名をstall_velocity_thresholdに修正 (`#148 <https://github.com/rt-net/crane_x7_ros/issues/148>`_)
* crane_x7_controlのREADMEにモード変更の注記を追加 (`#141 <https://github.com/rt-net/crane_x7_ros/issues/141>`_)
  * Update README.md
  * Update README.md
* Fix serial port settings description of crane_x7_control`s README. (`#140 <https://github.com/rt-net/crane_x7_ros/issues/140>`_)
  * Update README.md
  * Update README.md
* Support ROS Noetic (`#130 <https://github.com/rt-net/crane_x7_ros/issues/130>`_)
  * Add xacro prefix to macro tags
  * Fix example fortmats
  * update crane_x7_control CMakeLists.txt
  * update exmaple code formats
  * Update CI
* Update README.md for latency_timer (`#127 <https://github.com/rt-net/crane_x7_ros/issues/127>`_)
  * Update README.md
  * Update README.md
* Set position pid gain to zero for all joints at shutdown. (`#125 <https://github.com/rt-net/crane_x7_ros/issues/125>`_)
* Port images from the images branch to rt-net.github.io/images (`#120 <https://github.com/rt-net/crane_x7_ros/issues/120>`_)
  * Update README.md
  * Update README.en.md
  * Update README.md
  * Update README.md
  * Update README.md
  * Update README.en.md
* Fix data length of GOAL_VELOCITY (`#117 <https://github.com/rt-net/crane_x7_ros/issues/117>`_)
  * Fix data length of GOAL_VELOCITY
  * Add blank line at end of file
  * Delete diff of the No newline at end of file
* Merge pull request `#101 <https://github.com/rt-net/crane_x7_ros/issues/101>`_ from rt-net/fix\_`#100 <https://github.com/rt-net/crane_x7_ros/issues/100>`_
  Fix `#100 <https://github.com/rt-net/crane_x7_ros/issues/100>`_ for VC++.
* fix set_joint_param
* Fix commentout
* Merge pull request `#93 <https://github.com/rt-net/crane_x7_ros/issues/93>`_ from rt-net/melodic
  support ROS Melodic. Close `#84 <https://github.com/rt-net/crane_x7_ros/issues/84>`_
* Change parameter namespace
* Merge pull request `#87 <https://github.com/rt-net/crane_x7_ros/issues/87>`_ from rt-net/gazebo_with_pick_and_place
  Add pick and place example for crane_x7_gazebo. Close `#83 <https://github.com/rt-net/crane_x7_ros/issues/83>`_
* Delete pid gain parameter of finger_b_joint
* Add pick and place example for gazebo
* Fix conflicts
* Merge pull request `#72 <https://github.com/rt-net/crane_x7_ros/issues/72>`_ from rt-net/port_name\_`#71 <https://github.com/rt-net/crane_x7_ros/issues/71>`_
  Set dynamixel port name via launch arg. Close `#71 <https://github.com/rt-net/crane_x7_ros/issues/71>`_
* Fix conflicts
* Merge pull request `#85 <https://github.com/rt-net/crane_x7_ros/issues/85>`_ from rt-net/gazebo
  Add crane_x7_gazebo
* fix d gain 0 to 0.0 in gazebo_control.yaml
* Can control crane_x7 in gazebo with MoveIt.
* Merge pull request `#82 <https://github.com/rt-net/crane_x7_ros/issues/82>`_ from rt-net/fix\_`#73 <https://github.com/rt-net/crane_x7_ros/issues/73>`_
  Execute write_joint_param() once per a control loop. Close `#73 <https://github.com/rt-net/crane_x7_ros/issues/73>`_
* Execute write_joint_param() once per a control loop
* Merge pull request `#75 <https://github.com/rt-net/crane_x7_ros/issues/75>`_ from rt-net/teaching_sample
  Add Teaching sample
* Fix control commands of teaching_example
* Set dynamixel port name via launch arg
* Merge pull request `#65 <https://github.com/rt-net/crane_x7_ros/issues/65>`_ from rt-net/`#64 <https://github.com/rt-net/crane_x7_ros/issues/64>`__pid_gain_preset
  Add pid gain preset example. close `#64 <https://github.com/rt-net/crane_x7_ros/issues/64>`_
* Add comment to preset_reconfigure.py
* Add preset_pid_gain_example
* Merge pull request `#63 <https://github.com/rt-net/crane_x7_ros/issues/63>`_ from rt-net/`#62 <https://github.com/rt-net/crane_x7_ros/issues/62>`__modify_current_mode_README
  Modify joint name(README)
  close `#63 <https://github.com/rt-net/crane_x7_ros/issues/63>`_
* Modify joint name(README)
* Update package.xml
* 電流制御モードの取り扱い注意について明記
* Merge pull request `#56 <https://github.com/rt-net/crane_x7_ros/issues/56>`_ from rt-net/`#55 <https://github.com/rt-net/crane_x7_ros/issues/55>`__add_control_readme
  `#55 <https://github.com/rt-net/crane_x7_ros/issues/55>`_ add control readme
  close `#55 <https://github.com/rt-net/crane_x7_ros/issues/55>`_
* Add readme image
* Add readme
* Merge pull request `#53 <https://github.com/rt-net/crane_x7_ros/issues/53>`_ from rt-net/`#51 <https://github.com/rt-net/crane_x7_ros/issues/51>`__update_cmakelists
  Fix install in CMakeLists.txt, close `#51 <https://github.com/rt-net/crane_x7_ros/issues/51>`_
* Fix install in CMakeLists.txt
* Merge pull request `#50 <https://github.com/rt-net/crane_x7_ros/issues/50>`_ from rt-net/`#49 <https://github.com/rt-net/crane_x7_ros/issues/49>`__gripper_cmd_sample
  `#49 <https://github.com/rt-net/crane_x7_ros/issues/49>`_ gripper cmd sample
  close `#49 <https://github.com/rt-net/crane_x7_ros/issues/49>`_
* Modify gripper controller setting.
* Merge pull request `#43 <https://github.com/rt-net/crane_x7_ros/issues/43>`_ from rt-net/`#41 <https://github.com/rt-net/crane_x7_ros/issues/41>`__modify_finger_collision
  `#41 <https://github.com/rt-net/crane_x7_ros/issues/41>`_ modify finger collision
  close `#43 <https://github.com/rt-net/crane_x7_ros/issues/43>`_
* Merge pull request `#42 <https://github.com/rt-net/crane_x7_ros/issues/42>`_ from rt-net/`#16 <https://github.com/rt-net/crane_x7_ros/issues/16>`__load_only_once
  `#16 <https://github.com/rt-net/crane_x7_ros/issues/16>`_ load only once
  close `#16 <https://github.com/rt-net/crane_x7_ros/issues/16>`_
* Add constraints setting.
* Modify multiple load.
* Modify loading config namespace (using rosparam).
* Merge pull request `#39 <https://github.com/rt-net/crane_x7_ros/issues/39>`_ from rt-net/`#35 <https://github.com/rt-net/crane_x7_ros/issues/35>`__modify_startup_read
  Bug fix readpos function call mistakes.
  close `#35 <https://github.com/rt-net/crane_x7_ros/issues/35>`_
* Bug fix readpos function call mistakes.
* Merge pull request `#38 <https://github.com/rt-net/crane_x7_ros/issues/38>`_ from rt-net/`#18 <https://github.com/rt-net/crane_x7_ros/issues/18>`__add_pick_and_place
  Add demonstration package
* Fix proper noun, "CRANE-X7"
* Merge pull request `#23 <https://github.com/rt-net/crane_x7_ros/issues/23>`_ from rt-net/`#22 <https://github.com/rt-net/crane_x7_ros/issues/22>`__update_license
  Update license and author
* Update license and author
* Merge pull request `#13 <https://github.com/rt-net/crane_x7_ros/issues/13>`_ from rt-net/modify_control_dependencies
  Modify dependencies.
* Remove unused dependencies.
* Merge branch 'master' into collision_meshes
* Merge pull request `#12 <https://github.com/rt-net/crane_x7_ros/issues/12>`_ from rt-net/bulk_rw_optimization
  Bulk rw optimization
* Modify error log formats
* Change error log variable( string => queue) and publish timing,
* Bug fix
* Change lasterror publish timing
* Refactoring last_error variables
* Bug fix
* Optimization bulk read/write( pos/vel/curr rw => group rw)
* Merge pull request `#11 <https://github.com/rt-net/crane_x7_ros/issues/11>`_ from rt-net/servo_watch_dog
  Add bus watchdog setting to servo parameter.
* Delete comments
* Change index variables naming ( i=>ii, j=>jj)
* Refactoring - unused variables, variable declarations
* Add bus watchdog setting to servo parameter.
* Merge pull request `#10 <https://github.com/rt-net/crane_x7_ros/issues/10>`_ from rt-net/merge_control_package
  Merge control package
* Delete unused comments.
* Merge(rebase) joint_limits parameters.
* Modify changing rate function.
* Bug fixed.
* Modify switching rate code.
* Modify homing value type(cast to int32_t)
* Bug fixed.
* Bug fixed
* Add servo parameter setting functions.
* Debug create service instances.
  Debug spinner.
* Add dynamic reconfigure function.
* Modify write group name(Pos => Goal).
  Modify ope_mode function switch.
* Add read velocity fundction.
* Add effort constant parameter.
  Add operating mode parameter.
  Modify current setting function.
* Add effort limit function.
* Modify effort unit.
* Modify effort function.
* Add servo parameter.
  - effort const XM-430 = 1.79,  XM-540 = 2.79
  - operating mode (0:current mode, 3:position mode)
  Add read param function.
* Change file name.
* Revert "Change file name(commit miss recover)."
  This reverts commit 2bb6da5e9f592cc2a72e5b68d0c7d838de811364.
* Change file name(commit miss recover).
* Merge pull request `#4 <https://github.com/rt-net/crane_x7_ros/issues/4>`_ from rt-net/urdf_refactor
  Urdf refactor
* Modify changing rate function.
* Bug fixed.
* Modify switching rate code.
* Modify homing value type(cast to int32_t)
* Bug fixed.
* Bug fixed
* Add servo parameter setting functions.
* Debug create service instances.
  Debug spinner.
* Add dynamic reconfigure function.
* Modify write group name(Pos => Goal).
  Modify ope_mode function switch.
* Add read velocity fundction.
* Add effort constant parameter.
  Add operating mode parameter.
  Modify current setting function.
* Add effort limit function.
* Modify effort unit.
* Modify effort function.
* Add servo parameter.
  - effort const XM-430 = 1.79,  XM-540 = 2.79
  - operating mode (0:current mode, 3:position mode)
  Add read param function.
* Change file name.
* Revert "Change file name(commit miss recover)."
  This reverts commit 2bb6da5e9f592cc2a72e5b68d0c7d838de811364.
* Change file name(commit miss recover).
* Fix controller names and spaces
* Merge pull request `#9 <https://github.com/rt-net/crane_x7_ros/issues/9>`_ from rt-net/depend_gripper_action_controller
  Run-time dependency on gripper_action_controller package
* Merge pull request `#8 <https://github.com/rt-net/crane_x7_ros/issues/8>`_ from rt-net/use_ros_dynamixel_sdk
  Use dynamixel_sdk package provided by ROS
* Correct joint names
* Use xacro-generated URDF file
* Merge branch 'use_ros_dynamixel_sdk' into urdf_refactor
* Merge branch 'depend_gripper_action_controller' into urdf_refactor
* Run-time dependency on gripper_action_controller package
* Use dynamixel_sdk package provided by ROS
* Update joint names to match refactored URDF
* First commit.
* Contributors: Daisuke Sato, Geoffrey Biggs, RT Corp, Shota Aoki, ShotaAk, Tiryoh, nomumu, nomumuRT
