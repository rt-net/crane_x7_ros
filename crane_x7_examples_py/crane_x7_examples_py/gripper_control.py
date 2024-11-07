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

import math
import rclpy

# generic ros libraries
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

from crane_x7_examples_py.utils import plan_and_execute


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # ロガー生成
    logger = get_logger('pick_and_place')

    # MoveItPy初期化
    crane_x7 = MoveItPy(node_name='moveit_py')
    crane_x7_arm = crane_x7.get_planning_component('arm')
    crane_x7_gripper = crane_x7.get_planning_component('gripper')
    logger.info('MoveItPy instance created')

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_x7.get_robot_model()
    robot_state = RobotState(robot_model)

    # planningのパラメータ設定
    # armのパラメータ設定用
    arm_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )
    arm_plan_request_params.max_acceleration_scaling_factor \
        = 1.0  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor \
        = 1.0  # Set 0.0 ~ 1.0

    # gripperのパラメータ設定用
    gripper_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )
    gripper_plan_request_params.max_acceleration_scaling_factor \
        = 1.0  # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor \
        = 1.0  # Set 0.0 ~ 1.0

    # SRDFに定義されている'home'の姿勢にする
    crane_x7_arm.set_start_state_to_current_state()
    crane_x7_arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_x7,
        crane_x7_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperを60[deg]に開く
    robot_state.set_joint_group_positions('gripper', [math.radians(60)])
    crane_x7_gripper.set_start_state_to_current_state()
    crane_x7_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        crane_x7_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # gripperを0[deg]に閉じる
    robot_state.set_joint_group_positions('gripper', [math.radians(0)])
    crane_x7_gripper.set_start_state_to_current_state()
    crane_x7_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        crane_x7_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # gripperを60[deg]に開く
    robot_state.set_joint_group_positions('gripper', [math.radians(60)])
    crane_x7_gripper.set_start_state_to_current_state()
    crane_x7_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        crane_x7_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # gripperを0[deg]に閉じる
    robot_state.set_joint_group_positions('gripper', [math.radians(0)])
    crane_x7_gripper.set_start_state_to_current_state()
    crane_x7_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        crane_x7_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # MoveItPyの終了
    crane_x7.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == '__main__':
    main()
