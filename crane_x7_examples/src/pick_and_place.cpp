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

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  double GRIPPER_DEFAULT = 0.0;
  double GRIPPER_OPEN = angles::from_degrees(60.0);
  double GRIPPER_CLOSE = angles::from_degrees(20);

  // SRDFに定義されている"home"の姿勢にする
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // 何かを掴んでいた時のためにハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 可動範囲を制限する
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

  move_group_arm.setPathConstraints(constraints);

  // 掴む準備をする
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.3;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // 掴みに行く
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.13;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // ハンドを閉じる
  gripper_joint_values[0] = GRIPPER_CLOSE;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 持ち上げる
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.3;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // 移動する
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.3;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // 下ろす
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.13;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // ハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 少しだけハンドを持ち上げる
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.2;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(0), angles::from_degrees(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // 可動範囲の制限を解除
  move_group_arm.clearPathConstraints();

  // SRDFに定義されている"home"の姿勢にする
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // ハンドを閉じる
  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}
