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

class CartesianPath : public rclcpp::Node
{
public:
  // NodeOptionsを受け取るコンストラクタ
  explicit CartesianPath(const rclcpp::NodeOptions & options)
  : Node("cartesian_path", options) {}

  // アームとグリッパのMoveGroupInterfaceを初期化
  void initialize()
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");

    // 直線補間軌道の場合は、速度を小さく抑えて安全に駆動させます
    move_group_arm_->setMaxVelocityScalingFactor(0.1);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);
  }

  // SRDFに定義されている"home"の姿勢にするメソッド
  void moveToHome()
  {
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();
  }

  // グリッパ角度を目標角度（度）で指定して駆動するメソッド
  void setGripperAngle(double degrees)
  {
    auto gripper_joint_values = move_group_gripper_->getCurrentJointValues();
    gripper_joint_values[0] = angles::from_degrees(degrees);
    move_group_gripper_->setJointValueTarget(gripper_joint_values);
    move_group_gripper_->move();
  }

  // 円を描くような手先目標点を生成し、
  // 直線軌道計画（Cartesian Path）を計算・実行するメソッド
  void drawCircles(int repeat, double radius, double cx, double cy, double cz)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    float num_of_waypoints = 30;

    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    // 手先を垂直（真下）に向ける姿勢を設定
    q.setRPY(0, angles::from_degrees(180), 0);
    target_pose.orientation = tf2::toMsg(q);

    // 指定された繰り返し回数で円周上のwaypoint（経由点）を細かく作成
    for (int r = 0; r < repeat; r++) {
      for (int i = 0; i < num_of_waypoints; i++) {
        float theta = 2.0 * M_PI * (i / static_cast<float>(num_of_waypoints));
        target_pose.position.x = cx + radius * std::cos(theta);
        target_pose.position.y = cy + radius * std::sin(theta);
        target_pose.position.z = cz;
        waypoints.push_back(target_pose);
      }
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 直線経由点の間隔を1cmに設定して補間
    // waypointsを結ぶような手先直線補間軌道を計算
    move_group_arm_->computeCartesianPath(waypoints, eef_step, trajectory);
    // 計算された軌道を実行
    move_group_arm_->execute(trajectory);
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

  // CartesianPathクラスを実行
  auto node = std::make_shared<CartesianPath>(node_options);
  node->initialize();

  // アームを初期姿勢にする
  node->moveToHome();

  // ハンドを90度まで開く
  node->setGripperAngle(90.0);

  // 座標(x=0.3, y=0.0, z=0.1)を中心に、XY平面上に半径0.1 mの円を3回描くように動かす
  node->drawCircles(3, 0.1, 0.3, 0.0, 0.1);

  // アームを初期姿勢に戻す
  node->moveToHome();

  // ハンドを閉じる（0度）
  node->setGripperAngle(0.0);

  rclcpp::shutdown();
  return 0;
}
