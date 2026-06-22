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
// /5c15da709e9ea8529b54b313dc570f164f9a713e/doc/examples/subframes
// /src/subframes_tutorial.cpp

#include <memory>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PoseGroupstate : public rclcpp::Node
{
public:
  // NodeOptionsを受け取るコンストラクタ
  explicit PoseGroupstate(const rclcpp::NodeOptions & options)
  : Node("pose_groupstate", options)
  {
  }

  // MoveGroupInterfaceの初期化および速度・加速度の設定
  void initialize()
  {
    // MoveGroupInterfaceを初期化
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");

    // アームの最大速度および加速度をスケール値で調整（0.0〜1.0）
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);
  }

  // SRDFに定義されている'home'の姿勢にするメソッド
  void moveToHome()
  {
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();
  }

  // SRDFに定義されている'vertical'の姿勢にするメソッド
  void moveToVertical()
  {
    move_group_arm_->setNamedTarget("vertical");
    move_group_arm_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // 移動指令のためにバックグラウンドスピナーを準備
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // PoseGroupstateクラスを実行
  auto node = std::make_shared<PoseGroupstate>(node_options);
  node->initialize();

  // 'home'姿勢へ移動
  node->moveToHome();

  // 'vertical'姿勢へ移動
  node->moveToVertical();

  // 'home'姿勢に戻る
  node->moveToHome();

  rclcpp::shutdown();
  return 0;
}
