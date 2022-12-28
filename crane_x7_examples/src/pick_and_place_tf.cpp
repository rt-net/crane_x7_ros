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
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html

#include <chrono>
#include <cmath>
#include <memory>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    using namespace std::placeholders;
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);

    // SRDFに定義されている"home"の姿勢にする
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();

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

    move_group_arm_->setPathConstraints(constraints);

    // 待機姿勢
    control_arm(0.15, 0.0, 0.3, -180, 0, 90);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

private:
  void on_timer()
  {
    // 把持物体の位置を取得
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", "target",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to target: %s",
        ex.what());
      return;
    }

    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time now = system_clock.now();
    std::chrono::nanoseconds filtering_time = 1s;
    std::chrono::nanoseconds rest_time = 3s;
    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);

    // 現在時刻から1秒以内に受け取ったtfを使用
    if ((now.nanoseconds() - tf.stamp_.time_since_epoch().count()) < filtering_time.count()) {
      double tf_diff = (tf_past_.getOrigin() - tf.getOrigin()).length();
      // 把持物体の位置が止まっていることを判定
      if (tf_diff < 0.01) {
        // 3秒以上停止している場合ピッキング動作開始
        if ((now.nanoseconds() - tf_past_.stamp_.time_since_epoch().count()) > rest_time.count()) {
          picking(tf.getOrigin());
        }
      } else {
        tf_past_ = tf;
      }
    }
  }

  void picking(tf2::Vector3 target_position)
  {
    const double GRIPPER_DEFAULT_ = 0.0;
    const double GRIPPER_OPEN_ = angles::from_degrees(60.0);
    const double GRIPPER_CLOSE_ = angles::from_degrees(15.0);

    // 何かを掴んでいた時のためにハンドを開閉
    control_gripper(GRIPPER_OPEN_);
    control_gripper(GRIPPER_DEFAULT_);

    // 掴む準備をする
    control_arm(target_position.x(), target_position.y(), 0.2, -180, 0, 90);

    // ハンドを開く
    control_gripper(GRIPPER_OPEN_);

    // 掴みに行く
    control_arm(target_position.x(), target_position.y(), 0.13, -180, 0, 90);

    // ハンドを閉じる
    control_gripper(GRIPPER_CLOSE_);

    // 持ち上げる
    control_arm(target_position.x(), target_position.y(), 0.2, -180, 0, 90);

    // 移動する
    control_arm(0.2, 0.2, 0.2, -180, 0, 90);

    // 下ろす
    control_arm(0.2, 0.2, 0.13, -180, 0, 90);

    // ハンドを開く
    control_gripper(GRIPPER_OPEN_);

    // 少しだけハンドを持ち上げる
    control_arm(0.2, 0.2, 0.2, -180, 0, 90);

    // 待機姿勢に戻る
    control_arm(0.15, 0.0, 0.3, -180, 0, 90);

    // ハンドを閉じる
    control_gripper(GRIPPER_DEFAULT_);
  }

  void control_gripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

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
    move_group_arm_->setPoseTarget(target_pose);
    move_group_arm_->move();
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_place_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_arm_node,
    move_group_gripper_node);
  exec.add_node(pick_and_place_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
