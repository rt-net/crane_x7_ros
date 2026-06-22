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
    def __init__(self):
        # MoveItPyのインスタンス生成
        self.crane_x7 = MoveItPy(node_name='pick_and_place')
        self.logger = get_logger('pick_and_place')
        self.logger.info('MoveItPy instance created')

        # 各グループのplanning componentを取得
        self.arm = self.crane_x7.get_planning_component('arm')
        self.gripper = self.crane_x7.get_planning_component('gripper')

        # ロボットモデルオブジェクトの取得
        self.robot_model = self.crane_x7.get_robot_model()

        # 軌道計画用パラメータの初期化
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )
        self.gripper_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )

        # アームの動作速度の調整（0.0〜1.0）
        self.arm_plan_request_params.max_acceleration_scaling_factor = 1.0
        self.arm_plan_request_params.max_velocity_scaling_factor = 1.0

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

    def set_gripper_angle(self, degrees):
        # グリッパ角度を目標角度（度）で指定して駆動する
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [math.radians(degrees)])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_x7,
            self.gripper,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
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

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        self.arm.set_start_state_to_current_state()

        # オイラー角(ロール・ピッチ・ヨー)をクォータニオンメッセージに変換
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        quat_msg = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        target_pose = Pose(position=Point(x=x, y=y, z=z), orientation=quat_msg)

        # 目標位置姿勢を設定
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'crane_x7_mounting_plate_link'
        goal_pose.pose = target_pose
        self.arm.set_goal_state(
            pose_stamped_msg=goal_pose,
            pose_link='crane_x7_gripper_base_link',
        )
        plan_and_execute(
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )


def main(args=None):
    rclpy.init(args=args)

    # 制御クラスのインスタンスを生成
    node = PickAndPlace()

    # グリッパの開閉角度定義（度）
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = 60.0
    GRIPPER_CLOSE = 20.0

    # アームを初期姿勢にする
    node.move_to_home()

    # 以前掴んでいたもののためにハンドを開く
    node.set_gripper_angle(GRIPPER_OPEN)

    # アームの可動範囲制限を設定する
    node.set_constraints()

    # 1. 物体の上へ移動（アプローチ）
    node.move_arm_to_pose(0.2, 0.0, 0.3, -180.0, 0.0, -90.0)

    # 2. 物体を掴める高さへ降下
    node.move_arm_to_pose(0.2, 0.0, 0.13, -180.0, 0.0, -90.0)

    # 3. ハンドを閉じて物体を把持
    node.set_gripper_angle(GRIPPER_CLOSE)

    # 4. 物体を持ち上げる
    node.move_arm_to_pose(0.2, 0.0, 0.3, -180.0, 0.0, -90.0)

    # 5. 置く場所の上空まで移動する
    node.move_arm_to_pose(0.2, 0.2, 0.3, -180.0, 0.0, -90.0)

    # 6. 物体を降ろす
    node.move_arm_to_pose(0.2, 0.2, 0.13, -180.0, 0.0, -90.0)

    # 7. 物体をリリースする（ハンドを開く）
    node.set_gripper_angle(GRIPPER_OPEN)

    # 8. ハンドを少し持ち上げる（退避）
    node.move_arm_to_pose(0.2, 0.2, 0.2, -180.0, 0.0, -90.0)

    # 可動範囲制限を解除する
    node.clear_constraints()

    # 初期姿勢に戻る
    node.move_to_home()

    # グリッパを初期状態（閉じる）に戻す
    node.set_gripper_angle(GRIPPER_DEFAULT)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
