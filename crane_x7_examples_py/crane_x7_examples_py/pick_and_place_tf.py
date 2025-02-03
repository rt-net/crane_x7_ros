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

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # instantiate MoveItPy instance and get planning component
        self.crane_x7 = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # アーム制御用 planning component
        self.arm = self.crane_x7.get_planning_component('arm')
        # グリッパ制御用 planning component
        self.gripper = self.crane_x7.get_planning_component('gripper')

        # instantiate a RobotState instance using the current robot model
        self.robot_model = self.crane_x7.get_robot_model()

        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )
        self.gripper_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )

        # 動作速度の調整
        self.arm_plan_request_params.max_acceleration_scaling_factor = 0.7  # Set 0.0 ~ 1.0
        self.arm_plan_request_params.max_velocity_scaling_factor = 0.7  # Set 0.0 ~ 1.0

        self.gripper_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        # SRDFに定義されている'home'の姿勢にする
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_and_execute(
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

        # 可動範囲を制限する
        constraints = Constraints()
        constraints.name = 'arm_constraints'

        jointConstraint = JointConstraint()
        jointConstraint.joint_name = 'crane_x7_lower_arm_fixed_part_joint'
        jointConstraint.position = 0.0
        jointConstraint.tolerance_above = math.radians(30)
        jointConstraint.tolerance_below = math.radians(30)
        jointConstraint.weight = 1.0
        constraints.joint_constraints.append(jointConstraint)

        jointConstraint.joint_name = 'crane_x7_upper_arm_revolute_part_twist_joint'
        jointConstraint.position = 0.0
        jointConstraint.tolerance_above = math.radians(30)
        jointConstraint.tolerance_below = math.radians(30)
        jointConstraint.weight = 0.8
        constraints.joint_constraints.append(jointConstraint)

        self.arm.set_path_constraints(constraints)

        # 待機姿勢
        self.init_pose()

        # Call on_timer function every second
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のtf位置姿勢を取得
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'target_0', rclpy.time.Time())
        except TransformException as ex:
            self.logger.info(
                f'Could not transform base_link to target: {ex}'
                )
            return

        now_time = self.get_clock().now()
        FILTERING_TIME = rclpy.duration.Duration(seconds=2)
        STOP_TIME_THRESHOLD = rclpy.duration.Duration(seconds=3)
        DISTANCE_THRESHOLD = 0.01

        # 経過時間と停止時間を計算(nsec)
        # 経過時間
        tf_time = rclpy.time.Time.from_msg(tf_msg.header.stamp)
        TF_ELAPSED_TIME = now_time - tf_time
        # 停止時間
        tf_past_time = rclpy.time.Time.from_msg(self.tf_past.header.stamp)
        TF_STOP_TIME = now_time - tf_past_time
        TARGET_Z_MIN_LIMIT = 0.04

        # 現在時刻から2秒以内に受け取ったtfを使用
        if TF_ELAPSED_TIME < FILTERING_TIME:
            tf_diff = np.linalg.norm([
                self.tf_past.transform.translation.x - tf_msg.transform.translation.x,
                self.tf_past.transform.translation.y - tf_msg.transform.translation.y,
                self.tf_past.transform.translation.z - tf_msg.transform.translation.z
            ])
            # 把持対象の位置が停止していることを判定
            if tf_diff < DISTANCE_THRESHOLD:
                # 把持対象が3秒以上停止している場合ピッキング動作開始
                if TF_STOP_TIME > STOP_TIME_THRESHOLD:
                    # 把持対象が低すぎる場合は把持位置を調整
                    if tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT:
                        tf_msg.transform.translation.z = TARGET_Z_MIN_LIMIT
                    self._picking(tf_msg.transform.translation)
            else:
                self.tf_past = tf_msg

    def init_pose(self):
        joint_values = [
            math.radians(0.0),
            math.radians(90.0),
            math.radians(0.0),
            math.radians(-160.0),
            math.radians(0.0),
            math.radians(-50.0),
            math.radians(90.0)
        ]
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('arm', joint_values)
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
        )

    def _picking(self, target_position):
        GRIPPER_DEFAULT = 0.0
        GRIPPER_OPEN = math.radians(60.0)
        GRIPPER_CLOSE = math.radians(20.0)

        # 何かを掴んでいた時のためにハンドを開閉
        self._control_gripper(GRIPPER_OPEN)
        self._control_gripper(GRIPPER_DEFAULT)

        # 掴む準備をする
        self._control_arm(
            target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90)

        # ハンドを開く
        self._control_gripper(GRIPPER_OPEN)

        # 掴みに行く
        self._control_arm(
            target_position.x, target_position.y, target_position.z + 0.05, -180, 0, 90)

        # ハンドを閉じる
        self._control_gripper(GRIPPER_CLOSE)

        # 持ち上げる
        self._control_arm(
            target_position.x, target_position.y, target_position.z + 0.12, -180, 0, 90)

        # 移動する
        self._control_arm(0.1, 0.2, 0.2, -180, 0, 90)

        # 下ろす
        self._control_arm(0.1, 0.2, 0.13, -180, 0, 90)

        # ハンドを開く
        self._control_gripper(GRIPPER_OPEN)

        # 少しだけハンドを持ち上げる
        self._control_arm(0.1, 0.2, 0.2, -180, 0, 90)

        # 初期姿勢に戻る
        self.init_pose()

        # ハンドを閉じる
        self._control_gripper(GRIPPER_DEFAULT)

    # グリッパ制御
    def _control_gripper(self, angle):
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

    # アーム制御
    def _control_arm(self, x, y, z, roll, pitch, yaw):
        self.arm.set_start_state_to_current_state()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        self.arm.set_goal_state(
            pose_stamped_msg=goal_pose,
            pose_link='crane_x7_gripper_base_link'
        )
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

    rclpy.spin(pick_and_place_tf_node)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
