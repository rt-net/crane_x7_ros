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

#include <memory>
#include <vector>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class JointValues : public rclcpp::Node
{
public:
  // NodeOptionsを受け取るコンストラクタ
  explicit JointValues(const rclcpp::NodeOptions & options)
  : Node("joint_values", options) {}

  // MoveGroupInterfaceを初期化
  // shared_from_this()を使用するため、コンストラクタではなくこのメソッド内で初期化します
  void initialize()
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");

    // 駆動速度を調整する（0.0〜1.0）
    move_group_arm_->setMaxVelocityScalingFactor(0.5);
    move_group_arm_->setMaxAccelerationScalingFactor(0.5);
  }

  // SRDFに定義されている"vertical"の姿勢（すべての関節角度が0度）にするメソッド
  void moveToVertical()
  {
    move_group_arm_->setNamedTarget("vertical");
    move_group_arm_->move();
  }

  // 関節を一つずつ目標角度に変更して動作させるメソッド
  void rotateJointsSequentially()
  {
    // 現在の関節角度を取得
    auto joint_values = move_group_arm_->getCurrentJointValues();
    // 目標角度（-45度）をラジアンに変換
    double target_joint_value = angles::from_degrees(-45.0);

    // 各関節（ジョイント）の目標角度を一つずつ上書きして動作を指示する
    for (size_t i = 0; i < joint_values.size(); i++) {
      joint_values[i] = target_joint_value;
      move_group_arm_->setJointValueTarget(joint_values);
      move_group_arm_->move();
    }
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // JointValuesノードのインスタンスを作成
  auto node = std::make_shared<JointValues>(node_options);

  // MoveItはカレント状態の監視等のためにバックグラウンドでスピニング（スレッド処理）が必要です
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

  // 必要なインターフェースを初期化
  node->initialize();

  // 最初にアームを垂直の姿勢にする
  node->moveToVertical();

  // 各関節を順番に-45度へ動かす
  node->rotateJointsSequentially();

  // 最後にアームを垂直の姿勢に戻す
  node->moveToVertical();

  rclcpp::shutdown();
  return 0;
}
