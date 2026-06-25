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
#include <thread>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlace
{
public:
  // グリッパの開閉角度
  inline static const double GRIPPER_OPEN = angles::from_degrees(60.0);
  inline static const double GRIPPER_GRASP = angles::from_degrees(20.0);
  inline static const double GRIPPER_CLOSE = 0.0;

  // ノードを受け取り、アーム・グリッパのMoveGroupInterfaceを初期化する
  explicit PickAndPlace(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node, "gripper");
  }

  // アームを目標位置・姿勢（Pose）に動かす
  void move_arm_to_pose(const geometry_msgs::msg::Pose & pose)
  {
    move_group_arm_->setPoseTarget(pose);
    move_group_arm_->move();
  }

  // アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
  void control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);
    move_arm_to_pose(target_pose);
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // グリッパを角度[rad]を指定して開閉する
  void set_gripper_angle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // アームの関節の一部に可動制限を設定する
  void set_constraints()
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

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

  // 設定された関節可動制限をクリアする
  void clear_constraints()
  {
    move_group_arm_->clearPathConstraints();
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
  auto node = rclcpp::Node::make_shared("pick_and_place", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  PickAndPlace controller(node);

  // アームの目標姿勢（hand down姿勢: RPY = -180, 0, -90 [deg]）
  tf2::Quaternion q;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  geometry_msgs::msg::Pose pre_grasp_pose;
  pre_grasp_pose.position.x = 0.2;
  pre_grasp_pose.position.y = 0.0;
  pre_grasp_pose.position.z = 0.3;
  pre_grasp_pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
  grasp_pose.position.z = 0.13;

  geometry_msgs::msg::Pose pre_release_pose = pre_grasp_pose;
  pre_release_pose.position.y = 0.2;

  geometry_msgs::msg::Pose release_pose = pre_release_pose;
  release_pose.position.z = 0.13;

  geometry_msgs::msg::Pose post_release_pose = pre_release_pose;
  post_release_pose.position.z = 0.2;

  // 初期化動作
  controller.move_arm_to_named_pose("home");
  controller.set_gripper_angle(PickAndPlace::GRIPPER_OPEN);  // 何かを掴んでいた時のために開く
  controller.set_constraints();

  // ピック動作（掴みに行く）
  controller.move_arm_to_pose(pre_grasp_pose);   // 物体の上に腕を伸ばす
  controller.move_arm_to_pose(grasp_pose);        // アプローチ
  controller.set_gripper_angle(PickAndPlace::GRIPPER_GRASP);    // 掴む
  controller.move_arm_to_pose(pre_grasp_pose);    // 持ち上げる

  // プレース動作（移動して置く）
  controller.move_arm_to_pose(pre_release_pose);  // 移動する
  controller.move_arm_to_pose(release_pose);       // 下ろす
  controller.set_gripper_angle(PickAndPlace::GRIPPER_OPEN);      // 離す
  controller.move_arm_to_pose(post_release_pose);  // 少し持ち上げる

  // 終了動作
  controller.clear_constraints();
  controller.move_arm_to_named_pose("home");
  controller.set_gripper_angle(PickAndPlace::GRIPPER_CLOSE);

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
