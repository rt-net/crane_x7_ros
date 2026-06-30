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
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_x7 = MoveItPy(node_name='pose_groupstate')
        self.logger = get_logger('pose_groupstate')

        # アーム制御用 planning component
        self.arm = self.crane_x7.get_planning_component('arm')

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(self.crane_x7, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    def move_arm_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名でアームを動かす
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration_name)
        plan_and_execute(
            self.crane_x7, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )


def main(args=None):
    rclpy.init(args=args)

    controller = PoseGroupstate()

    # SRDFに定義された名前付き姿勢を順番に動かす
    controller.move_arm_to_named_pose('home')
    controller.move_arm_to_named_pose('vertical')
    controller.move_arm_to_named_pose('home')

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
