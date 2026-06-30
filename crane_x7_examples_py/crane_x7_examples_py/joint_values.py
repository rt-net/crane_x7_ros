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
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_x7 = MoveItPy(node_name='joint_values')
        self.logger = get_logger('joint_values')

        # アーム制御用 planning component
        self.arm = self.crane_x7.get_planning_component('arm')

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.crane_x7.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(self.crane_x7, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 0.5  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 0.5  # Set 0.0 ~ 1.0

    def move_arm_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名でアームを動かす
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration_name)
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def move_arm_joint_values(self, joint_values_dict):
        # 各ジョイント角度[rad]を指定してアームを動かす
        # joint_values_dictはジョイント名をキー、角度[rad]を値とする辞書
        robot_state = RobotState(self.robot_model)
        robot_state.joint_positions = joint_values_dict

        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=self.robot_model.get_joint_model_group('arm'),
        )

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def get_current_arm_joint_values(self):
        # アームの現在のジョイント角度を辞書形式で取得する
        current_state = self.arm.get_start_state()
        return current_state.get_joint_group_positions('arm')


def main(args=None):
    rclpy.init(args=args)

    controller = JointValues()

    joint_names = [
        'crane_x7_shoulder_fixed_part_pan_joint',
        'crane_x7_shoulder_revolute_part_tilt_joint',
        'crane_x7_upper_arm_revolute_part_twist_joint',
        'crane_x7_upper_arm_revolute_part_rotate_joint',
        'crane_x7_lower_arm_fixed_part_joint',
        'crane_x7_lower_arm_revolute_part_joint',
        'crane_x7_wrist_joint',
    ]
    target_angle = math.radians(-45.0)

    # verticalの姿勢にする（すべてのジョイントの目標角度が0度になる）
    controller.move_arm_to_named_pose('vertical')

    # 各ジョイントを順番に-45[deg]に動かす
    joint_values = controller.get_current_arm_joint_values()
    joint_values_dict = dict(zip(joint_names, joint_values))

    for joint_name in joint_names:
        joint_values_dict[joint_name] = target_angle
        controller.move_arm_joint_values(joint_values_dict)

    # 垂直に戻す
    controller.move_arm_to_named_pose('vertical')

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
