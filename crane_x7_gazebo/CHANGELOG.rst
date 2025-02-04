^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package crane_x7_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Gazeboのclockをtopicへ配信 (`#192 <https://github.com/rt-net/crane_x7_ros/issues/192>`_)
  ros_gz_bridge追加
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
* Merge pull request `#158 <https://github.com/rt-net/crane_x7_ros/issues/158>`_ from rt-net/remove_crane_x7_gazebo_config
  crane_x7_gazeboのコンフィグファイルを削除
* crane_x7_gazeboのconfigファイルを削除
* Merge pull request `#157 <https://github.com/rt-net/crane_x7_ros/issues/157>`_ from rt-net/ros2-devel
  Support ROS 2 Foxy
* robot_description_lodarの仕様変更に対応
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
* roboticsgroup_gazebo_pluginsをroboticsgroup_upatras_gazebo_pluginsに変更する (`#146 <https://github.com/rt-net/crane_x7_ros/issues/146>`_)
  * グリッパのプラグインにroboticsgroup_upatras_gazebo_pluginsを使用する
  * roboticsgroupのプラグインはaptでインストールできるため、READMEとCIからパッケージクローンのコマンドを削除
* Fix CMakeLists.txt (`#139 <https://github.com/rt-net/crane_x7_ros/issues/139>`_)
* Disable shadows in gazebo world (`#135 <https://github.com/rt-net/crane_x7_ros/issues/135>`_)
* Merge pull request `#88 <https://github.com/rt-net/crane_x7_ros/issues/88>`_ from rt-net/fix\_`#86 <https://github.com/rt-net/crane_x7_ros/issues/86>`_
  Update package.xml. Close `#86 <https://github.com/rt-net/crane_x7_ros/issues/86>`_
* Fix an author name and add dependencies to crane_x7_gazebo/package.xml
* Merge pull request `#87 <https://github.com/rt-net/crane_x7_ros/issues/87>`_ from rt-net/gazebo_with_pick_and_place
  Add pick and place example for crane_x7_gazebo. Close `#83 <https://github.com/rt-net/crane_x7_ros/issues/83>`_
* Add pick and place example for gazebo
* Fix conflicts
* Fix conflicts
* Merge pull request `#85 <https://github.com/rt-net/crane_x7_ros/issues/85>`_ from rt-net/gazebo
  Add crane_x7_gazebo
* Update CMakeLists.txt
* Can control crane_x7 in gazebo with MoveIt.
* Add CRANE-X7 model to gazebo
* Add crane_x7_gazebo. Gazebo display a table
* Contributors: Shota Aoki, ShotaAk
