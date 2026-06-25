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

import time


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    # 軌道を計画する
    logger.info('Planning trajectory')
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    # 計画した軌道を実行する
    if plan_result:
        logger.info('Executing plan')
        robot_trajectory = plan_result.trajectory
        execute_result = robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error('Planning failed')
        execute_result = False

    time.sleep(sleep_time)
    return execute_result
