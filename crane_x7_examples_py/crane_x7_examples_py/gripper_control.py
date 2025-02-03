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


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('gripper_control')

    # instantiate MoveItPy instance and get planning component
    crane_x7 = MoveItPy(node_name='gripper_control')
    logger.info('MoveItPy instance created')

    # アーム制御用 planning component
    arm = crane_x7.get_planning_component('arm')
    # グリッパ制御用 planning component
    gripper = crane_x7.get_planning_component('gripper')

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_x7.get_robot_model()

    arm_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )
    gripper_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )

    # 動作速度の調整
    arm_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    gripper_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # SRDFに定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperを60[deg]に開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(60.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # gripperを0[deg]に閉じる
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(0.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # gripperを60[deg]に開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(60.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # gripperを0[deg]に閉じる
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(0.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_x7,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
