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

from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


class JointValues:
    def __init__(self):
        # MoveItPyのインスタンス生成とロガーの取得
        self.crane_x7 = MoveItPy(node_name='joint_values')
        self.logger = get_logger('joint_values')
        self.logger.info('MoveItPy instance created')

        # アーム制御用のplanning componentを取得
        self.arm = self.crane_x7.get_planning_component('arm')

        # ロボットモデルからロボット状態オブジェクトを作成
        self.robot_model = self.crane_x7.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        # 軌道計画用パラメータの設定
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )

        # 動作速度の調整（0.0〜1.0）
        self.arm_plan_request_params.max_acceleration_scaling_factor = 0.5
        self.arm_plan_request_params.max_velocity_scaling_factor = 0.5

    def move_to_vertical(self):
        # SRDFに定義されている'vertical'の姿勢にする
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='vertical')
        plan_and_execute(
            self.crane_x7,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

    def rotate_joints_sequentially(self):
        # 対象となる各関節の名前リスト
        joint_names = [
            'crane_x7_shoulder_fixed_part_pan_joint',
            'crane_x7_shoulder_revolute_part_tilt_joint',
            'crane_x7_upper_arm_revolute_part_twist_joint',
            'crane_x7_upper_arm_revolute_part_rotate_joint',
            'crane_x7_lower_arm_fixed_part_joint',
            'crane_x7_lower_arm_revolute_part_joint',
            'crane_x7_wrist_joint',
        ]
        # 目標角度 -45度をラジアンに変換
        target_joint_value = math.radians(-45.0)

        # 現在角度をベースに、目標角度用の状態を作成する
        current_state = self.arm.get_start_state()
        joint_values = current_state.get_joint_group_positions('arm')

        # リストをdict(関節名: 角度値)に変換する
        joint_values_dict = dict(zip(joint_names, joint_values))

        # 各関節角度を順番に目標値へ変更して実行する
        for joint_name in joint_names:
            joint_values_dict[joint_name] = target_joint_value
            self.robot_state.joint_positions = joint_values_dict

            # MoveItの関節拘束（制約）オブジェクトを構築
            joint_constraint = construct_joint_constraint(
                robot_state=self.robot_state,
                joint_model_group=self.crane_x7.get_robot_model().get_joint_model_group('arm'),
            )

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(motion_plan_constraints=[joint_constraint])

            plan_and_execute(
                self.crane_x7,
                self.arm,
                self.logger,
                single_plan_parameters=self.arm_plan_request_params,
            )


def main(args=None):
    rclpy.init(args=args)

    # 制御クラスのインスタンスを生成
    node = JointValues()

    # 最初にアームを垂直の姿勢にする
    node.move_to_vertical()

    # 各関節を順番に目標角度（-45度）へ動かす
    node.rotate_joints_sequentially()

    # 最後にアームを垂直の姿勢に戻す
    node.move_to_vertical()

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
