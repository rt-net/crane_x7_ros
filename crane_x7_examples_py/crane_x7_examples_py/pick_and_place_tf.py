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

import math

from crane_x7_examples_py.utils import plan_and_execute

from geometry_msgs.msg import Pose, PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import Constraints, JointConstraint

import numpy as np

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException, TransformListener, TransformStamped
from tf2_ros.buffer import Buffer


class PickAndPlaceTf(Node):
    # グリッパの開閉角度
    GRIPPER_OPEN = math.radians(60.0)
    GRIPPER_GRASP = math.radians(20.0)
    GRIPPER_CLOSE = math.radians(0.0)

    # 置く位置（プレース位置）のXYZ[m]とRPY[deg]
    PLACE_X = 0.1
    PLACE_Y = 0.2
    PLACE_Z = 0.13
    PLACE_ROLL = -180.0
    PLACE_PITCH = 0.0
    PLACE_YAW = 90.0
    PLACE_APPROACH_Z = 0.2

    def __init__(self):
        super().__init__('pick_and_place_tf')
        self.logger = self.get_logger()

        # TFリスナーの初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_x7 = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # アーム・グリッパ制御用 planning component
        self.arm = self.crane_x7.get_planning_component('arm')
        self.gripper = self.crane_x7.get_planning_component('gripper')

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.crane_x7.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(self.crane_x7, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 0.7  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 0.7  # Set 0.0 ~ 1.0

        self.gripper_plan_params = PlanRequestParameters(self.crane_x7, 'ompl_rrtc')

        # homeの姿勢にしてから待機姿勢に移行する
        self.move_arm_to_named_pose('home')
        self.init_pose()

        # 0.5秒ごとにon_timerを呼び出すタイマーを作成
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のtf位置姿勢を取得
        try:
            tf_msg = self.tf_buffer.lookup_transform('base_link', 'target_0', rclpy.time.Time())
        except TransformException as ex:
            self.logger.info(f'Could not transform base_link to target: {ex}')
            return

        now = self.get_clock().now()
        FILTERING_TIME = rclpy.duration.Duration(seconds=2)
        STOP_TIME_THRESHOLD = rclpy.duration.Duration(seconds=3)
        DISTANCE_THRESHOLD = 0.01
        TARGET_Z_MIN_LIMIT = 0.04

        tf_elapsed_time = now - rclpy.time.Time.from_msg(tf_msg.header.stamp)
        tf_stop_time = now - rclpy.time.Time.from_msg(self.tf_past.header.stamp)

        # 現在時刻から2秒以内に受け取ったtfを使用
        if tf_elapsed_time > FILTERING_TIME:
            return

        tf_diff = np.linalg.norm(
            [
                self.tf_past.transform.translation.x - tf_msg.transform.translation.x,
                self.tf_past.transform.translation.y - tf_msg.transform.translation.y,
                self.tf_past.transform.translation.z - tf_msg.transform.translation.z,
            ]
        )

        # 把持対象の位置が停止していることを判定
        if tf_diff > DISTANCE_THRESHOLD:
            self.tf_past = tf_msg
            return

        # 把持対象が3秒以上停止している場合ピッキング動作開始
        if tf_stop_time < STOP_TIME_THRESHOLD:
            return

        # 把持対象が低すぎる場合は把持位置を調整
        if tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT:
            tf_msg.transform.translation.z = TARGET_Z_MIN_LIMIT

        self.picking(tf_msg.transform.translation)

    def init_pose(self):
        # カメラで把持対象を撮影するための待機姿勢に移動する
        joint_values = [
            math.radians(0.0),
            math.radians(90.0),
            math.radians(0.0),
            math.radians(-160.0),
            math.radians(0.0),
            math.radians(-50.0),
            math.radians(90.0),
        ]
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('arm', joint_values)
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def picking(self, target_position):
        # 何かを掴んでいた時のためにハンドを開閉
        self.move_gripper_angle(self.GRIPPER_OPEN)
        self.move_gripper_angle(self.GRIPPER_CLOSE)

        # ピック動作（掴みに行く）
        self.control_arm(
            target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90
        )
        self.move_gripper_angle(self.GRIPPER_OPEN)
        self.control_arm(
            target_position.x, target_position.y, target_position.z + 0.05, -180, 0, 90
        )
        self.move_gripper_angle(self.GRIPPER_GRASP)
        self.control_arm(
            target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90
        )

        # プレース動作（移動して置く）
        self.control_arm(
            self.PLACE_X, self.PLACE_Y, self.PLACE_APPROACH_Z,
            self.PLACE_ROLL, self.PLACE_PITCH, self.PLACE_YAW)
        self.control_arm(
            self.PLACE_X, self.PLACE_Y, self.PLACE_Z,
            self.PLACE_ROLL, self.PLACE_PITCH, self.PLACE_YAW)
        self.move_gripper_angle(self.GRIPPER_OPEN)
        self.control_arm(
            self.PLACE_X, self.PLACE_Y, self.PLACE_APPROACH_Z,
            self.PLACE_ROLL, self.PLACE_PITCH, self.PLACE_YAW)

        # 待機姿勢に戻る
        self.init_pose()
        self.move_gripper_angle(self.GRIPPER_CLOSE)

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

    pick_and_place_tf_node = PickAndPlaceTf()
    pick_and_place_tf_node.set_constraints()

    rclpy.spin(pick_and_place_tf_node)

    pick_and_place_tf_node.clear_constraints()

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
