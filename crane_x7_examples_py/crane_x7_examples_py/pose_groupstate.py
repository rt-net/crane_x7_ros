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

from crane_x7_examples_py.utils import plan_and_execute

from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


class PoseGroupstate:
    def __init__(self):
        # MoveItPyのインスタンス生成
        self.crane_x7 = MoveItPy(node_name='pose_groupstate')
        self.logger = get_logger('pose_groupstate')
        self.logger.info('MoveItPy instance created')

        # アーム制御用のplanning componentを取得
        self.arm = self.crane_x7.get_planning_component('arm')

        # 軌道計画パラメータの設定
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_x7,
            'ompl_rrtc',
        )

        # 動作速度の調整（0.0〜1.0）
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


def main(args=None):
    rclpy.init(args=args)

    # 制御クラスのインスタンスを生成
    node = PoseGroupstate()

    # 'home'姿勢へ移動
    node.move_to_home()

    # 'vertical'姿勢へ移動
    node.move_to_vertical()

    # 'home'姿勢に戻る
    node.move_to_home()

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
