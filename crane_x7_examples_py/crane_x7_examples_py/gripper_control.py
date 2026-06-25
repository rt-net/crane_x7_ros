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

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


class GripperControl:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_x7 = MoveItPy(node_name='gripper_control')
        self.logger = get_logger('gripper_control')

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

    def move_arm_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名でアームを動かす
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration_name)
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def set_gripper_angle(self, angle):
        # グリッパを角度[rad]を指定して開閉する
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [angle])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_x7, self.gripper, self.logger,
            single_plan_parameters=self.gripper_plan_params,
        )


def main(args=None):
    rclpy.init(args=args)

    controller = GripperControl()

    # homeの姿勢にする
    controller.move_arm_to_named_pose('home')

    # グリッパを開閉する
    controller.set_gripper_angle(math.radians(60.0))
    controller.set_gripper_angle(math.radians(0.0))
    controller.set_gripper_angle(math.radians(60.0))
    controller.set_gripper_angle(math.radians(0.0))

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
