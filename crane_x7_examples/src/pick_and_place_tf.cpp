// Copyright 2022 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node, rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    // アームとグリッパのMoveGroupInterfaceを初期化
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.7);
    move_group_arm_->setMaxAccelerationScalingFactor(0.7);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");

    // tf バッファとリスナーのセットアップ
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // SRDFに定義されている"home"の姿勢にするメソッド
  void moveToHome()
  {
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();
  }

  // アームの各関節に動作範囲の制約をかけるメソッド
  void setConstraints()
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    // 一部の関節角度範囲を制限することで、ロボットに予期しない挙動（反転など）を防ぐ
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(30);
    joint_constraint.tolerance_below = angles::from_degrees(30);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(30);
    joint_constraint.tolerance_below = angles::from_degrees(30);
    joint_constraint.weight = 0.8;
    constraints.joint_constraints.push_back(joint_constraint);

    move_group_arm_->setPathConstraints(constraints);
  }

  // 関節負荷の低い待機撮影姿勢にするメソッド
  void initPose()
  {
    std::vector<double> joint_values = {
      angles::from_degrees(0.0), angles::from_degrees(90.0), angles::from_degrees(0.0),
      angles::from_degrees(-160.0), angles::from_degrees(0.0), angles::from_degrees(-50.0),
      angles::from_degrees(90.0),
    };
    move_group_arm_->setJointValueTarget(joint_values);
    move_group_arm_->move();
  }

  // 定期的な物体検出タイマーを開始するメソッド
  void startTimer()
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&PickAndPlaceTf::onTimer, this));
  }

private:
  void onTimer()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    // 検出対象（target_0）のbase_linkから見た位置姿勢をTFから取得
    try {
      tf_msg = tf_buffer_->lookupTransform("base_link", "target_0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform base_link to target: %s", ex.what());
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const auto FILTERING_TIME = rclcpp::Duration(2s);
    const auto STOP_TIME_THRESHOLD = rclcpp::Duration(3s);
    const double DISTANCE_THRESHOLD = 0.01;
    const double TARGET_Z_MIN_LIMIT = 0.04;

    // 最新メッセージの経過時間と、停止している時間の測定
    const auto tf_elapsed_time = now - rclcpp::Time(tf_msg.header.stamp);
    const auto tf_stop_time = now - rclcpp::Time(tf_past_.header.stamp);

    // 現在時刻から2秒以上古いTF情報は信頼できないため無視する
    if (tf_elapsed_time > FILTERING_TIME) {
      return;
    }

    // 過去の位置情報との距離を計算
    const double dx = tf_past_.transform.translation.x - tf_msg.transform.translation.x;
    const double dy = tf_past_.transform.translation.y - tf_msg.transform.translation.y;
    const double dz = tf_past_.transform.translation.z - tf_msg.transform.translation.z;
    const double tf_diff = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 把持対象オブジェクトが移動している場合は位置を更新して戻る
    if (tf_diff > DISTANCE_THRESHOLD) {
      tf_past_ = tf_msg;
      return;
    }

    // オブジェクトが3秒以上安定して停止している場合にのみピッキングを開始する
    if (tf_stop_time < STOP_TIME_THRESHOLD) {
      return;
    }

    // オブジェクト位置が低すぎる場合は、把持位置を安全な閾値下限（0.04m）に調整
    if (tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT) {
      tf_msg.transform.translation.z = TARGET_Z_MIN_LIMIT;
    }

    // ピッキング動作を実行
    picking(tf_msg.transform.translation);
  }

  void picking(geometry_msgs::msg::Vector3 target_position)
  {
    const double GRIPPER_DEFAULT = 0.0;
    const double GRIPPER_OPEN = angles::from_degrees(60.0);
    const double GRIPPER_CLOSE = angles::from_degrees(20.0);

    // すでに何かを掴んでいた場合のためにハンドを開閉して状態リセット
    controlGripper(GRIPPER_OPEN);
    controlGripper(GRIPPER_DEFAULT);

    // 把持対象の上空へ移動してアプローチ準備
    controlArm(target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90);

    // ハンドを開く
    controlGripper(GRIPPER_OPEN);

    // 把持位置へ降下
    controlArm(target_position.x, target_position.y, target_position.z + 0.07, -180, 0, 90);

    // ハンドを閉じて物体を把持
    controlGripper(GRIPPER_CLOSE);

    // 物体を上に持ち上げる
    controlArm(target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90);

    // プレース位置の上空へ移動
    controlArm(0.1, 0.2, 0.2, -180, 0, 90);

    // プレース位置へ下ろす
    controlArm(0.1, 0.2, 0.13, -180, 0, 90);

    // ハンドを開いて物体をリリース
    controlGripper(GRIPPER_OPEN);

    // ハンドを安全に少し持ち上げる
    controlArm(0.1, 0.2, 0.2, -180, 0, 90);

    // 待機撮影姿勢に戻る
    initPose();

    // ハンドを初期状態（閉じる）に戻す
    controlGripper(GRIPPER_DEFAULT);
  }

  // グリッパを目標角度（ラジアン）に駆動するメソッド
  void controlGripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // グリッパ（手先リンク）の目標位置姿勢を指定してアームを制御するメソッド
  void controlArm(
    const double x, const double y, const double z, const double roll, const double pitch,
    const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    // ロール、ピッチ、ヨー角度からクォータニオンを生成
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);
    move_group_arm_->setPoseTarget(target_pose);
    move_group_arm_->move();
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  geometry_msgs::msg::TransformStamped tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // タイマーコールバックとMoveGroupInterfaceの処理を並行させるためMultiThreadedExecutorを使用
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<PickAndPlaceTf>(move_group_arm_node, move_group_gripper_node);
  executor.add_node(node);
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([&executor]() {executor.spin();});

  // 初期姿勢（home）に移動
  node->moveToHome();

  // アームの可動範囲制限を設定
  node->setConstraints();

  // 関節負荷の低い待機撮影姿勢にする
  node->initPose();

  // 定期的な位置判定のためのタイマーを開始
  node->startTimer();

  // executorがシャットダウンされるまで待機
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
