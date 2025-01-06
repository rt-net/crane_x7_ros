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

from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_rsp_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    ld = LaunchDescription()

    description_loader = RobotDescriptionLoader()

    ld.add_action(
        DeclareLaunchArgument(
            'loaded_description',
            default_value=description_loader.load(),
            description='Set robot_description text.  \
                        It is recommended to use RobotDescriptionLoader() in  \
                            crane_x7_description.',
        )
    )

    moveit_config = (
        MoveItConfigsBuilder('crane_x7')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        'robot_description': LaunchConfiguration('loaded_description')
    }

    # Move group
    ld.add_entity(generate_move_group_launch(moveit_config))

    # RViz
    ld.add_entity(generate_moveit_rviz_launch(moveit_config))

    # Static TF
    ld.add_entity(generate_static_virtual_joint_tfs_launch(moveit_config))

    # Publish TF
    ld.add_entity(generate_rsp_launch(moveit_config))

    return ld
