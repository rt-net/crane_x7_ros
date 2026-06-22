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
        # MoveItPyのインスタンス生成
        self.crane_x7 = MoveItPy(node_name='gripper_control')
        self.logger = get_logger('gripper_control')
        self.logger.info('MoveItPy instance created')

        # アーム制御用とグリッパ制御用のplanning componentを取得
        self.arm = self.crane_x7.get_planning_component('arm')
        self.gripper = self.crane_x7.get_planning_component('gripper')

        # ロボットモデルオブジェクトの取得
        self.robot_model = self.crane_x7.get_robot_model()

        # 各グループの計画要求パラメータの初期化
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )
        self.gripper_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )

        # アーム動作速度の調整（0.0〜1.0）
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
        # グリッパを目標値（ラジアンに変換）にして駆動させる
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


def main(args=None):
    rclpy.init(args=args)

    # 制御クラスのインスタンスを生成
    node = GripperControl()

    # 初期姿勢に移動
    node.move_to_home()

    # グリッパを任意の角度（度）で開閉する
    node.set_gripper_angle(60.0)
    node.set_gripper_angle(0.0)
    node.set_gripper_angle(60.0)
    node.set_gripper_angle(0.0)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
