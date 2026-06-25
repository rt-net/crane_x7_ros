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

#include <cmath>
#include <memory>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlace : public rclcpp::Node
{
public:
  // NodeOptionsを受け取るコンストラクタ
  explicit PickAndPlace(const rclcpp::NodeOptions & options)
  : Node("pick_and_place", options) {}

  // アームとグリッパのMoveGroupInterfaceを初期化
  void initialize()
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");

    // アーム駆動速度の最大スケーリング（0.0〜1.0）
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);
  }

  // SRDFに定義されている"home"の姿勢にするメソッド
  void moveToHome()
  {
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();
  }

  // グリッパ角度を目標角度（度）で指定して制御するメソッド
  void setGripperAngle(double degrees)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angles::from_degrees(degrees);
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
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

  // 設定した動作制約をクリアするメソッド
  void clearConstraints() {move_group_arm_->clearPathConstraints();}

  // ロボットのグリッパ（手先リンク）の位置姿勢を指定して制御するメソッド
  void moveArmToPose(double x, double y, double z, double roll, double pitch, double yaw)
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

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // バックグラウンドで状態監視用のノードを動かす
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // 実処理を行うPickAndPlaceクラスをインスタンス化して実行
  auto node = std::make_shared<PickAndPlace>(node_options);
  node->initialize();

  const double GRIPPER_DEFAULT = 0.0;
  const double GRIPPER_OPEN = 60.0;
  const double GRIPPER_CLOSE = 20.0;

  // アームを初期姿勢にする
  node->moveToHome();

  // 以前掴んでいたもののために、念のためハンドを開く
  node->setGripperAngle(GRIPPER_OPEN);

  // アームの関節可動範囲にソフトな制約を設定する
  node->setConstraints();

  // 1. 物体の上へ移動する（アプローチ）
  node->moveArmToPose(0.2, 0.0, 0.3, -180.0, 0.0, -90.0);

  // 2. 物体のある高さまで降下する
  node->moveArmToPose(0.2, 0.0, 0.13, -180.0, 0.0, -90.0);

  // 3. 物体を掴む（グリッパを閉じる）
  node->setGripperAngle(GRIPPER_CLOSE);

  // 4. 物体を持ち上げる
  node->moveArmToPose(0.2, 0.0, 0.3, -180.0, 0.0, -90.0);

  // 5. 物体を運ぶ目標位置の上空へ移動する
  node->moveArmToPose(0.2, 0.2, 0.3, -180.0, 0.0, -90.0);

  // 6. 物体を降ろす
  node->moveArmToPose(0.2, 0.2, 0.13, -180.0, 0.0, -90.0);

  // 7. 物体をリリースする（グリッパを開く）
  node->setGripperAngle(GRIPPER_OPEN);

  // 8. アームを少し持ち上げる（退避動作）
  node->moveArmToPose(0.2, 0.2, 0.2, -180.0, 0.0, -90.0);

  // 可動範囲の制約設定を解除する
  node->clearConstraints();

  // 初期姿勢に戻す
  node->moveToHome();

  // ハンドを初期状態（閉じる）に戻す
  node->setGripperAngle(GRIPPER_DEFAULT);

  rclcpp::shutdown();
  return 0;
}
