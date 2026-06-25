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

from geometry_msgs.msg import PoseStamped

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
    def __init__(self):
        super().__init__('pick_and_place_tf')
        self.logger = self.get_logger()

        # tf バッファとリスナーのセットアップ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # MoveItPyのインスタンス生成
        self.crane_x7 = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # アームとグリッパの planning component を取得
        self.arm = self.crane_x7.get_planning_component('arm')
        self.gripper = self.crane_x7.get_planning_component('gripper')

        # ロボットモデルオブジェクトを取得
        self.robot_model = self.crane_x7.get_robot_model()

        # 軌道計画用パラメータの設定
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )
        self.gripper_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )

        # 動作速度の調整（0.0〜1.0）
        self.arm_plan_request_params.max_acceleration_scaling_factor = 0.7
        self.arm_plan_request_params.max_velocity_scaling_factor = 0.7

        self.timer = None

    def move_to_home(self):
        # SRDFに定義されている'home'の姿勢にする
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_and_execute(
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

    def set_constraints(self):
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

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'crane_x7_upper_arm_revolute_part_twist_joint'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(30)
        joint_constraint.tolerance_below = math.radians(30)
        joint_constraint.weight = 0.8
        constraints.joint_constraints.append(joint_constraint)

        self.arm.set_path_constraints(constraints)

    def start_timer(self):
        # 定期的な位置判定のために0.5秒おきにタイマーを実行する
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # 検出対象（target_0）のカメラから見た位置姿勢をTFから取得
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

        # 最新メッセージの経過時間と、停止している時間の測定
        tf_elapsed_time = now - rclpy.time.Time.from_msg(tf_msg.header.stamp)
        tf_stop_time = now - rclpy.time.Time.from_msg(self.tf_past.header.stamp)

        # 現在時刻から2秒以上古いTF情報は信頼できないため無視する
        if tf_elapsed_time > FILTERING_TIME:
            return

        # 過去の位置情報との距離を計算
        tf_diff = np.linalg.norm(
            [
                self.tf_past.transform.translation.x - tf_msg.transform.translation.x,
                self.tf_past.transform.translation.y - tf_msg.transform.translation.y,
                self.tf_past.transform.translation.z - tf_msg.transform.translation.z,
            ]
        )

        # 把持対象オブジェクトが停止している（前回の検出からの移動が少ない）ことを判定
        if tf_diff > DISTANCE_THRESHOLD:
            self.tf_past = tf_msg
            return

        # オブジェクトが3秒以上安定して停止している場合にのみピッキングを開始する
        if tf_stop_time < STOP_TIME_THRESHOLD:
            return

        # オブジェクト位置が低すぎる場合は、把持位置を安全な閾値下限（0.04m）に調整
        if tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT:
            tf_msg.transform.translation.z = TARGET_Z_MIN_LIMIT

        # ピッキング動作を実行
        self.picking(tf_msg.transform.translation)

    def init_pose(self):
        # 関節負荷の低い待機用の関節角度を設定
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
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

    def picking(self, target_position):
        GRIPPER_DEFAULT = 0.0
        GRIPPER_OPEN = math.radians(60.0)
        GRIPPER_CLOSE = math.radians(20.0)

        # すでに何かを掴んでいた場合のためにハンドを開閉して状態リセット
        self.control_gripper(GRIPPER_OPEN)
        self.control_gripper(GRIPPER_DEFAULT)

        # 把持対象の上空へ移動してアプローチ準備
        self.control_arm(
            target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90
        )

        # ハンドを開く
        self.control_gripper(GRIPPER_OPEN)

        # 把持位置へ降下
        self.control_arm(
            target_position.x, target_position.y, target_position.z + 0.05, -180, 0, 90
        )

        # ハンドを閉じて物体を把持
        self.control_gripper(GRIPPER_CLOSE)

        # 物体を上に持ち上げる
        self.control_arm(
            target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90
        )

        # プレース位置の上空へ移動
        self.control_arm(0.1, 0.2, 0.2, -180, 0, 90)

        # プレース位置へ下ろす
        self.control_arm(0.1, 0.2, 0.13, -180, 0, 90)

        # ハンドを開いて物体をリリース
        self.control_gripper(GRIPPER_OPEN)

        # ハンドを安全に少し持ち上げる
        self.control_arm(0.1, 0.2, 0.2, -180, 0, 90)

        # 待機撮影姿勢に戻る
        self.init_pose()

        # ハンドを初期状態（閉じる）に戻す
        self.control_gripper(GRIPPER_DEFAULT)

    def control_gripper(self, angle):
        # グリッパを目標角度（ラジアン）に駆動する
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [angle])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_x7,
            self.gripper,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
        )

    def control_arm(self, x, y, z, roll, pitch, yaw):
        # グリッパ（手先リンク）の目標位置姿勢を指定してアームを制御する
        self.arm.set_start_state_to_current_state()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        # ロール、ピッチ、ヨーからクォータニオンを生成して設定
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        self.arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='crane_x7_gripper_base_link')
        result = plan_and_execute(
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )
        return result


def main(args=None):
    rclpy.init(args=args)

    pick_and_place_tf_node = PickAndPlaceTf()

    # 初期姿勢（home）に移動
    pick_and_place_tf_node.move_to_home()

    # アームの可動範囲制限を設定
    pick_and_place_tf_node.set_constraints()

    # 関節負荷の低い待機撮影姿勢にする
    pick_and_place_tf_node.init_pose()

    # 定期的な位置判定のためのタイマーを開始
    pick_and_place_tf_node.start_timer()

    rclpy.spin(pick_and_place_tf_node)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
