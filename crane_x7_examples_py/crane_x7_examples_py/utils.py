# Copyright 2020 RT Corporation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import numpy as np
import quaternion

# generic ros libraries
from rclpy.logging import get_logger


# plan and execute
def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    '''Helper function to plan and execute a motion.'''
    # plan to goal
    logger = get_logger('plan_and_execute')
    logger.info('Planning trajectory')
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    result = None
    if plan_result:
        logger.info('Executing plan')
        robot_trajectory = plan_result.trajectory
        result = robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error('Planning failed')
        result = False
    time.sleep(sleep_time)
    return result


# euler --> quaternion
def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)

    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp

    return [qx, qy, qz, qw]


# rotation matrix --> quaternion
def rotation_matrix_to_quaternion(rotation_matrix):
    # numpy-quaternionを使用して回転行列からクォータニオンを計算
    # 3x3の回転行列をnumpy.quaternionに変換する
    q = quaternion.from_rotation_matrix(rotation_matrix)
    return q
