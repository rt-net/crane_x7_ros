# Copyright 2025 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
import math

from crane_x7_examples_py.utils import plan_and_execute

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import Constraints, JointConstraint

import rclpy
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('pick_and_place')

    # instantiate MoveItPy instance and get planning component
    crane_x7 = MoveItPy(node_name='pick_and_place')
    logger.info('MoveItPy instance created')

    # アーム制御用 planning component
    arm = crane_x7.get_planning_component('arm')
    # グリッパ制御用 planning component
    gripper = crane_x7.get_planning_component('gripper')

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_x7.get_robot_model()

    arm_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )
    gripper_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )

    # 動作速度の調整
    arm_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    gripper_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # グリッパの開閉角度
    GRIPPER_CLOSE = 0.0
    GRIPPER_OPEN = math.radians(60.0)
    GRIPPER_GRASP = math.radians(20.0)
    # 物体を持ち上げる高さ
    LIFTING_HEIFHT = 0.3
    # 物体を掴む位置
    gripper_quat = Rotation.from_euler('xyz', [-180.0, 0.0, -90.0], degrees=True).as_quat()
    gripper_quat_msg = Quaternion(
        x=gripper_quat[0], y=gripper_quat[1], z=gripper_quat[2], w=gripper_quat[3]
    )
    GRASP_POSE = Pose(position=Point(x=0.2, y=0.0, z=0.13), orientation=gripper_quat_msg)
    PRE_AND_POST_GRASP_POSE = copy.deepcopy(GRASP_POSE)
    PRE_AND_POST_GRASP_POSE.position.z = LIFTING_HEIFHT
    # 物体を置く位置
    RELEASE_POSE = Pose(position=Point(x=0.2, y=0.2, z=0.13), orientation=gripper_quat_msg)
    PRE_AND_POST_RELEASE_POSE = copy.deepcopy(RELEASE_POSE)
    PRE_AND_POST_RELEASE_POSE.position.z = LIFTING_HEIFHT

    # SRDFに定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_OPEN])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 可動範囲を制限する
    constraints = Constraints()
    constraints.name = 'arm_constraints'

    joint_constraint = JointConstraint()
    joint_constraint.joint_name = 'crane_x7_lower_arm_fixed_part_joint'
    joint_constraint.position = 0.0
    joint_constraint.tolerance_above = math.radians(30)
    joint_constraint.tolerance_below = math.radians(30)
    joint_constraint.weight = 1.0
    constraints.joint_constraints.append(joint_constraint)

    joint_constraint.joint_name = 'crane_x7_upper_arm_revolute_part_twist_joint'
    joint_constraint.position = 0.0
    joint_constraint.tolerance_above = math.radians(30)
    joint_constraint.tolerance_below = math.radians(30)
    joint_constraint.weight = 0.8
    constraints.joint_constraints.append(joint_constraint)

    arm.set_path_constraints(constraints)

    # 物体の上に腕を伸ばす
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
    goal_pose.pose = PRE_AND_POST_GRASP_POSE
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link='crane_x7_gripper_base_link',
    )
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 掴みに行く
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
    goal_pose.pose = GRASP_POSE
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link='crane_x7_gripper_base_link',
    )
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # ハンドを閉じる
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_GRASP])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 持ち上げる
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
    goal_pose.pose = PRE_AND_POST_GRASP_POSE
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link='crane_x7_gripper_base_link',
    )
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 移動する
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
    goal_pose.pose = PRE_AND_POST_RELEASE_POSE
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link='crane_x7_gripper_base_link',
    )
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 下ろす
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
    goal_pose.pose = RELEASE_POSE
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link='crane_x7_gripper_base_link',
    )
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # ハンドを開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_OPEN])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # ハンドを持ち上げる
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
    goal_pose.pose = PRE_AND_POST_RELEASE_POSE
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link='crane_x7_gripper_base_link',
    )
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # SRDFに定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # ハンドを閉じる
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_CLOSE])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
