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
#include <chrono>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf: public rclcpp::Node
{
  public:
    PickAndPlaceTf(rclcpp::Node::SharedPtr move_group_arm_node, rclcpp::Node::SharedPtr move_group_gripper_node)
    //PickAndPlaceTf()
    : Node("pick_and_place_tf")
    {
      using namespace std::placeholders;
      move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
      move_group_arm_->setMaxVelocityScalingFactor(1.0);
      move_group_arm_->setMaxAccelerationScalingFactor(1.0);

      move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
      move_group_gripper_->setMaxVelocityScalingFactor(1.0);
      move_group_gripper_->setMaxAccelerationScalingFactor(1.0);

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

      tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Call on_timer function every second
      timer_ = this->create_wall_timer(
        500ms, std::bind(&PickAndPlaceTf::on_timer, this));
    }

  private:
    void on_timer()
    {
      geometry_msgs::msg::TransformStamped tf_msg;

      try {
        tf_msg = tf_buffer_->lookupTransform(
          "target", "base_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform base_link to target: %s",
          ex.what());
        return;
      }
    }

    std::shared_ptr<MoveGroupInterface> move_group_arm_;
    std::shared_ptr<MoveGroupInterface> move_group_gripper_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto demo_node = std::make_shared<PickAndPlaceTf>(move_group_arm_node, move_group_gripper_node);
  exec.add_node(demo_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
