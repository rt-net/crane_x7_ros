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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches   \
    import generate_static_virtual_joint_tfs_launch
from moveit_configs_utils.launches import generate_rsp_launch


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    ld = LaunchDescription()

    declare_loaded_description = DeclareLaunchArgument(
        'loaded_description',
        default_value='',
        description='Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in  \
                        crane_x7_description.',
    )

    ld.add_action(declare_loaded_description)

    ld.add_action(DeclareBooleanLaunchArg('debug', default_value=False))
    
    ld.add_action(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=get_package_share_directory('crane_x7_moveit_config')
            + '/config/moveit.rviz',
            description='Set the path to rviz configuration file.',
        )
    )

    moveit_config = (
        MoveItConfigsBuilder('crane_x7')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('crane_x7_description'),
                'urdf',
                'crane_x7.urdf.xacro',
            ),
            mappings={},
        )
        .robot_description_semantic(
            file_path='config/crane_x7.srdf',
            mappings={'model': 'crane_x7'},
        )
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(
            file_path='config/controllers.yaml', moveit_manage_controllers=True
        )
        .planning_pipelines(pipelines=['ompl'])
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        'robot_description': LaunchConfiguration('loaded_description')
    }

    moveit_config.move_group_capabilities = {
        'capabilities': ''
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
