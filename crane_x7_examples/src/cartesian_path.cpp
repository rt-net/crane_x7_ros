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
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class CartesianPath
{
public:
  // ノードを受け取り、アーム・グリッパのMoveGroupInterfaceを初期化する
  explicit CartesianPath(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.1);  // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node, "gripper");
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // グリッパを角度[rad]を指定して開閉する
  void move_gripper_angle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // 経由点リスト（waypoints）に沿って手先を直交座標系で動かす
  void execute_cartesian_path(const std::vector<geometry_msgs::msg::Pose> & waypoints)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    move_group_arm_->computeCartesianPath(waypoints, eef_step, trajectory);
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
  auto node = rclcpp::Node::make_shared("cartesian_path", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  CartesianPath controller(node);

  // homeの姿勢にする
  controller.move_arm_to_named_pose("home");
  controller.move_gripper_angle(angles::from_degrees(90));

  // 座標(x=0.3, y=0.0, z=0.1)を中心にXY平面上で半径0.1 mの円を3回描く経由点を生成する
  std::vector<geometry_msgs::msg::Pose> waypoints;
  const int NUM_WAYPOINTS = 30;
  const int REPEAT = 3;
  const double RADIUS = 0.1;
  const double CENTER_X = 0.3;
  const double CENTER_Y = 0.0;
  const double CENTER_Z = 0.1;

  tf2::Quaternion q;
  q.setRPY(0, angles::from_degrees(180), 0);
  geometry_msgs::msg::Pose waypoint;
  waypoint.orientation = tf2::toMsg(q);

  for (int r = 0; r < REPEAT; r++) {
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
      double theta = 2.0 * M_PI * (i / static_cast<double>(NUM_WAYPOINTS));
      waypoint.position.x = CENTER_X + RADIUS * std::cos(theta);
      waypoint.position.y = CENTER_Y + RADIUS * std::sin(theta);
      waypoint.position.z = CENTER_Z;
      waypoints.push_back(waypoint);
    }
  }

  // 生成した経由点に沿って手先を動かす
  controller.execute_cartesian_path(waypoints);

  // homeの姿勢に戻る
  controller.move_arm_to_named_pose("home");
  controller.move_gripper_angle(0);

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
