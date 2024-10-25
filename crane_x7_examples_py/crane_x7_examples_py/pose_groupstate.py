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

import rclpy

# generic ros libraries
from rclpy.logging import get_logger

# moveit python library
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from crane_x7_examples_py.utils import plan_and_execute


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # ロガー生成
    logger = get_logger("pose_groupstate")

    # MoveItPy初期化
    crane_x7 = MoveItPy(node_name="moveit_py")
    crane_x7_arm = crane_x7.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    # planningのパラメータ設定
    plan_request_params = PlanRequestParameters(
        crane_x7,
        "ompl_rrtc",
    )

    # 速度＆加速度のスケーリングファクタを設定
    plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # 現在の位置から"home"ポジションに動かす
    crane_x7_arm.set_start_state_to_current_state()
    crane_x7_arm.set_goal_state(configuration_name="home")
    plan_and_execute(
        crane_x7,
        crane_x7_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 現在の位置から"vertical"ポジションに動かす
    crane_x7_arm.set_start_state_to_current_state()
    crane_x7_arm.set_goal_state(configuration_name="vertical")
    plan_and_execute(
        crane_x7,
        crane_x7_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 現在の位置から"home"ポジションに動かす
    crane_x7_arm.set_start_state_to_current_state()
    crane_x7_arm.set_goal_state(configuration_name="home")
    plan_and_execute(
        crane_x7,
        crane_x7_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # MoveItPyの終了
    crane_x7.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == "__main__":
    main()
