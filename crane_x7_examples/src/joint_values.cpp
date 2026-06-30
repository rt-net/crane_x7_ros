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

#include <thread>
#include <vector>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class JointValues
{
public:
  // ノードを受け取り、アームのMoveGroupInterfaceを初期化する
  explicit JointValues(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(0.5);  // Set 0.0 ~ 1.0
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // 各ジョイント角度[rad]を指定してアームを動かす
  void move_arm_joint_values(const std::vector<double> & joint_values)
  {
    move_group_arm_->setJointValueTarget(joint_values);
    move_group_arm_->move();
  }

  // アームの現在のジョイント角度を取得する
  std::vector<double> get_current_arm_joint_values()
  {
    return move_group_arm_->getCurrentJointValues();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("joint_values", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  JointValues controller(node);

  // verticalの姿勢にする（すべてのジョイントの目標角度が0度になる）
  controller.move_arm_to_named_pose("vertical");

  // 各ジョイントを順番に-45[deg]に動かす
  const double target_angle = angles::from_degrees(-45.0);
  auto joint_values = controller.get_current_arm_joint_values();
  for (size_t i = 0; i < joint_values.size(); i++) {
    joint_values[i] = target_angle;
    controller.move_arm_joint_values(joint_values);
  }

  // 垂直に戻す
  controller.move_arm_to_named_pose("vertical");

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
