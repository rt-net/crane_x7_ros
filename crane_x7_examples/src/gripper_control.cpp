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
// https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes
// /run_move_group/src/run_move_group.cpp

#include <cmath>
#include <memory>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GripperControl : public rclcpp::Node
{
public:
  // NodeOptionsを受け取るコンストラクタ
  explicit GripperControl(const rclcpp::NodeOptions & options)
  : Node("gripper_control", options)
  {
  }

  // アームとグリッパのMoveGroupInterfaceを初期化
  void initialize()
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");

    // アームの駆動速度調整（0.0〜1.0）
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);
  }

  // SRDFに定義されている"home"の姿勢にアームを移動するメソッド
  void moveToHome()
  {
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();
  }

  // グリッパの開閉角度を目標角度（度）で指定して駆動するメソッド
  void setGripperAngle(double degrees)
  {
    auto gripper_joint_values = move_group_gripper_->getCurrentJointValues();
    gripper_joint_values[0] = angles::from_degrees(degrees);
    move_group_gripper_->setJointValueTarget(gripper_joint_values);
    move_group_gripper_->move();
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

  // GripperControlノードのノードインスタンス（アーム用、グリッパ用）を定義
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // スレッドExecutorのセットアップ
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // 実処理ノードとしてGripperControlを実行
  auto node = std::make_shared<GripperControl>(node_options);
  node->initialize();

  // アームを初期姿勢に移動
  node->moveToHome();

  // グリッパを指定の角度（度）で開閉する
  node->setGripperAngle(60.0);
  node->setGripperAngle(0.0);
  node->setGripperAngle(60.0);
  node->setGripperAngle(0.0);

  rclcpp::shutdown();
  return 0;
}
