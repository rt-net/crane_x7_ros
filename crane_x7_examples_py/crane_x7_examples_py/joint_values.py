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


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('joint_values')

    # instantiate MoveItPy instance and get planning component
    crane_x7 = MoveItPy(node_name='joint_values')
    logger.info('MoveItPy instance created')

    # アーム制御用 planning component
    arm = crane_x7.get_planning_component('arm')

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_x7.get_robot_model()
    robot_state = RobotState(robot_model)

    arm_plan_request_params = PlanRequestParameters(
        crane_x7,
        'ompl_rrtc',
    )

    # 動作速度の調整
    arm_plan_request_params.max_acceleration_scaling_factor = 0.5  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 0.5  # Set 0.0 ~ 1.0

    # SRDFに定義されている'vertical'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    joint_names = [
        'crane_x7_shoulder_fixed_part_pan_joint',
        'crane_x7_shoulder_revolute_part_tilt_joint',
        'crane_x7_upper_arm_revolute_part_twist_joint',
        'crane_x7_upper_arm_revolute_part_rotate_joint',
        'crane_x7_lower_arm_fixed_part_joint',
        'crane_x7_lower_arm_revolute_part_joint',
        'crane_x7_wrist_joint',
        ]
    target_joint_value = math.radians(-45.0)

    # 各関節角度を順番に-45[deg]ずつ動かす
    for joint_name in joint_names:
        arm.set_start_state_to_current_state()

        joint_values = {joint_name: target_joint_value}
        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=crane_x7.get_robot_model().get_joint_model_group('arm'),
        )
        arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        plan_and_execute(
            crane_x7,
            arm,
            logger,
            single_plan_parameters=arm_plan_request_params,
        )

    # SRDFに定義されている'vertical'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        crane_x7,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
