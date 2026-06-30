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


class PickAndPlace:
    # グリッパの開閉角度
    GRIPPER_OPEN = math.radians(60.0)
    GRIPPER_GRASP = math.radians(20.0)
    GRIPPER_CLOSE = math.radians(0.0)

    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_x7 = MoveItPy(node_name='pick_and_place')
        self.logger = get_logger('pick_and_place')

        # アーム・グリッパ制御用 planning component
        self.arm = self.crane_x7.get_planning_component('arm')
        self.gripper = self.crane_x7.get_planning_component('gripper')

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.crane_x7.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(self.crane_x7, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.gripper_plan_params = PlanRequestParameters(self.crane_x7, 'ompl_rrtc')

    def move_arm_to_pose(self, pose):
        # アームを目標位置・姿勢（Pose）に動かす
        # 座標系はcrane_x7_mounting_plate_link、目標リンクはcrane_x7_gripper_base_link
        self.arm.set_start_state_to_current_state()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
        goal_pose.pose = pose
        self.arm.set_goal_state(
            pose_stamped_msg=goal_pose,
            pose_link='crane_x7_gripper_base_link',
        )
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def control_arm(self, x, y, z, roll, pitch, yaw):
        # アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.move_arm_to_pose(pose)

    def move_arm_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名でアームを動かす
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration_name)
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def move_gripper_angle(self, angle):
        # グリッパを角度[rad]を指定して開閉する
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [angle])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_x7, self.gripper, self.logger,
            single_plan_parameters=self.gripper_plan_params,
        )

    def set_constraints(self):
        # アームの関節の一部に可動制限を設定する
        constraints = Constraints()
        constraints.name = 'arm_constraints'

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'crane_x7_lower_arm_fixed_part_joint'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(30)
        joint_constraint.tolerance_below = math.radians(30)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'crane_x7_upper_arm_revolute_part_twist_joint'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(30)
        joint_constraint.tolerance_below = math.radians(30)
        joint_constraint.weight = 0.8
        constraints.joint_constraints.append(joint_constraint)

        self.arm.set_path_constraints(constraints)

    def clear_constraints(self):
        # 設定された関節可動制限をクリアする
        self.arm.clear_path_constraints()


def main(args=None):
    rclpy.init(args=args)

    controller = PickAndPlace()

    # 物体を持ち上げる高さ
    LIFTING_HEIGHT = 0.3

    # アームの目標姿勢（hand down姿勢: RPY = -180, 0, -90 [deg]）
    gripper_quat = Rotation.from_euler('xyz', [-180.0, 0.0, -90.0], degrees=True).as_quat()
    gripper_quat_msg = Quaternion(
        x=gripper_quat[0], y=gripper_quat[1], z=gripper_quat[2], w=gripper_quat[3]
    )
    grasp_pose = Pose(position=Point(x=0.2, y=0.0, z=0.13), orientation=gripper_quat_msg)
    pre_grasp_pose = copy.deepcopy(grasp_pose)
    pre_grasp_pose.position.z = LIFTING_HEIGHT

    release_pose = Pose(position=Point(x=0.2, y=0.2, z=0.13), orientation=gripper_quat_msg)
    pre_release_pose = copy.deepcopy(release_pose)
    pre_release_pose.position.z = LIFTING_HEIGHT

    # 初期化動作
    controller.move_arm_to_named_pose('home')
    controller.move_gripper_angle(controller.GRIPPER_OPEN)  # 何かを掴んでいた時のために開く
    controller.set_constraints()

    # ピック動作（掴みに行く）
    controller.move_arm_to_pose(pre_grasp_pose)   # 物体の上に腕を伸ばす
    controller.move_arm_to_pose(grasp_pose)        # アプローチ
    controller.move_gripper_angle(controller.GRIPPER_GRASP)    # 掴む
    controller.move_arm_to_pose(pre_grasp_pose)    # 持ち上げる

    # プレース動作（移動して置く）
    controller.move_arm_to_pose(pre_release_pose)  # 移動する
    controller.move_arm_to_pose(release_pose)       # 下ろす
    controller.move_gripper_angle(controller.GRIPPER_OPEN)      # 離す
    controller.move_arm_to_pose(pre_release_pose)   # 少し持ち上げる

    # 終了動作
    controller.clear_constraints()
    controller.move_arm_to_named_pose('home')
    controller.move_gripper_angle(controller.GRIPPER_CLOSE)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
