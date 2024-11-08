# Copyright 2020 RT Corporation
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

import datetime
import math

from geometry_msgs.msg import Pose

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import Constraints, JointConstraint

import numpy as np

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformListener, TransformStamped
from tf2_ros.buffer import Buffer

from crane_x7_examples_py.utils import euler_to_quaternion, plan_and_execute


class PickAndPlaceTf(Node):
    def __init__(self, crane_x7):
        super().__init__('pick_and_place_tf_node')
        self.logger = get_logger('pick_and_place_tf')
        self.tf_past = TransformStamped()
        self.crane_x7 = crane_x7
        self.crane_x7_arm = crane_x7.get_planning_component('arm')
        self.crane_x7_gripper = crane_x7.get_planning_component('gripper')
        # instantiate a RobotState instance using the current robot model
        self.robot_model = crane_x7.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        # planningのパラメータ設定
        # armのパラメータ設定用
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )
        # Set 0.0 ~ 1.0
        self.arm_plan_request_params.max_velocity_scaling_factor = 0.7

        # Set 0.0 ~ 1.0
        self.arm_plan_request_params.max_acceleration_scaling_factor = 0.7

        # gripperのパラメータ設定用
        self.gripper_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )
        # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_velocity_scaling_factor = 1.0

        # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_acceleration_scaling_factor = 1.0

        # SRDFに定義されている'home'の姿勢にする
        self.crane_x7_arm.set_start_state_to_current_state()
        self.crane_x7_arm.set_goal_state(configuration_name='home')
        plan_and_execute(
            self.crane_x7,
            self.crane_x7_arm,
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

        jointConstraint.joint_name \
            = 'crane_x7_upper_arm_revolute_part_twist_joint'
        jointConstraint.position = 0.0
        jointConstraint.tolerance_above = math.radians(30)
        jointConstraint.tolerance_below = math.radians(30)
        jointConstraint.weight = 0.8
        constraints.joint_constraints.append(jointConstraint)

        self.crane_x7_arm.set_path_constraints(constraints)

        # 関節への負荷が低い撮影姿勢
        self.init_pose()

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のtf位置姿勢を取得
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'target_0', Time())
        except tf2_ros.LookupException as ex:
            self.get_logger().info(
                f'Could not transform base_link to target: {ex}'
                )

        now = Time()
        FILTERING_TIME = datetime.timedelta(seconds=2)
        STOP_TIME_THRESHOLD = datetime.timedelta(seconds=3)
        DISTANCE_THRESHOLD = 0.01
        TARGET_Z_MIN_LIMIT = 0.04

        # 経過時間と停止時間を計算(nsec)
        # 経過時間
        TF_ELAPSED_TIME = now.nanoseconds - tf_msg.header.stamp.nanosec
        # 停止時間
        if self.tf_past is not None:
            TF_STOP_TIME = now.nanoseconds - self.tf_past.header.stamp.nanosec
        else:
            TF_STOP_TIME = now.nanoseconds

        # 現在時刻から2秒以内に受け取ったtfを使用
        if TF_ELAPSED_TIME < FILTERING_TIME.total_seconds() * 1e9:
            tf_diff = np.sqrt((self.tf_past.transform.translation.x
                               - tf_msg.transform.translation.x) ** 2
                              + (self.tf_past.transform.translation.y
                              - tf_msg.transform.translation.y) ** 2
                              + (self.tf_past.transform.translation.z
                              - tf_msg.transform.translation.z) ** 2
                              )
            # 把持対象の位置が停止していることを判定
            if tf_diff < DISTANCE_THRESHOLD:
                # 把持対象が3秒以上停止している場合ピッキング動作開始
                if TF_STOP_TIME > STOP_TIME_THRESHOLD.total_seconds() * 1e9:
                    # 把持対象が低すぎる場合は把持位置を調整
                    if tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT:
                        tf_msg.transform.translation.z = TARGET_Z_MIN_LIMIT
                    self._picking(tf_msg)
            else:
                self.tf_past = tf_msg

    def init_pose(self):
        joint_values = []
        joint_values.append(math.radians(0.0))
        joint_values.append(math.radians(90.0))
        joint_values.append(math.radians(0.0))
        joint_values.append(math.radians(-160.0))
        joint_values.append(math.radians(0.0))
        joint_values.append(math.radians(-50.0))
        joint_values.append(math.radians(90.0))
        self.robot_state.set_joint_group_positions('arm', joint_values)
        self.crane_x7_arm.set_start_state_to_current_state()
        self.crane_x7_arm.set_goal_state(robot_state=self.robot_state)
        plan_and_execute(
            self.crane_x7,
            self.crane_x7_arm,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
        )

    def _picking(self, tf_msg):
        GRIPPER_DEFAULT = 0.0
        GRIPPER_OPEN = math.radians(60.0)
        GRIPPER_CLOSE = math.radians(20.0)

        # 何かを掴んでいた時のためにハンドを開閉
        self._control_gripper(GRIPPER_OPEN)
        self._control_gripper(GRIPPER_DEFAULT)

        # 掴む準備をする
        self._control_arm(
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z + 0.12,
            -180, 0, 90)

        # ハンドを開く
        self._control_gripper(GRIPPER_OPEN)

        # 掴みに行く
        self._control_arm(
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z + 0.07,
            -180, 0, 90)

        # ハンドを閉じる
        self._control_gripper(GRIPPER_CLOSE)

        # 持ち上げる
        self._control_arm(
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z + 0.12,
            -180, 0, 90)

        # 移動する
        self._control_arm(0.1, 0.2, 0.2, -180, 0, 90)

        # 下ろす
        self._control_arm(0.1, 0.2, 0.13, -180, 0, -90)

        # ハンドを開く
        self._control_gripper(GRIPPER_OPEN)

        # 少しだけハンドを持ち上げる
        self._control_arm(0.1, 0.2, 0.2, -180, 0, 90)

        # 待機姿勢に戻る
        self._control_arm(0.0, 0.0, 0.17, 0, 0, 0)

        # 初期姿勢に戻る
        self.init_pose()

        # ハンドを閉じる
        self._control_gripper(GRIPPER_DEFAULT)

    # グリッパ制御
    def _control_gripper(self, angle):
        self.robot_state.set_joint_group_positions('gripper', [angle])
        self.crane_x7_gripper.set_start_state_to_current_state()
        self.crane_x7_gripper.set_goal_state(robot_state=self.robot_state)
        plan_and_execute(
            self.crane_x7,
            self.crane_x7_gripper,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
        )

    # アーム制御
    def _control_arm(self, x, y, z, roll, pitch, yaw):
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        q = euler_to_quaternion(math.radians(roll), math.radians(pitch),
                                math.radians(yaw))
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        self.crane_x7_arm.set_start_state_to_current_state()
        self.crane_x7_arm.set_goal_state(
            pose_stamped_msg=target_pose, pose_link='crane_x7_wrist_link'
        )
        result = plan_and_execute(
            self.crane_x7,
            self.crane_x7_arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )
        return result


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # MoveItPy初期化
    crane_x7 = MoveItPy(node_name='moveit_py')

    # node生成
    pick_and_place_tf_node = PickAndPlaceTf(crane_x7)
    rclpy.spin(pick_and_place_tf_node)

    # MoveItPyの終了
    crane_x7.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == '__main__':
    main()