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
from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'), 'config', 'manipulator_config.yaml'
    )

    links_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'), 'config', 'manipulator_links.csv'
    )

    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Set port name.'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='3000000',
        description='Set baudrate.'
    )

    declare_timeout_seconds = DeclareLaunchArgument(
        'timeout_seconds',
        default_value='1.0',
        description='Set timeout seconds.'
    )

    declare_manipulator_config_file_path = DeclareLaunchArgument(
        'manipulator_config_file_path',
        default_value=config_file_path,
        description='Set manipulator config file path.'
    )

    declare_manipulator_links_file_path = DeclareLaunchArgument(
        'manipulator_links_file_path',
        default_value=links_file_path,
        description='Set manipulator links file path.'
    )

    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Use gazebo or not.'
    )

    declare_use_d435 = DeclareLaunchArgument(
        'use_d435',
        default_value='false',
        description='Use d435 or not.'
    )

    declare_use_mock_components = DeclareLaunchArgument(
        'use_mock_components',
        default_value='false',
        description='Use mock_components or not.'
    )

    declare_gz_control_config_package = DeclareLaunchArgument(
        'gz_control_config_package',
        default_value='',
        description='Set gz control config package.'
    )

    declare_gz_control_config_file_path = DeclareLaunchArgument(
        'gz_control_config_file_path',
        default_value='',
        description='Set gz control config file path.'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description_loader.baudrate = LaunchConfiguration('baudrate')
    description_loader.timeout_seconds = LaunchConfiguration('timeout_seconds')
    description_loader.use_gazebo = LaunchConfiguration('use_gazebo')
    description_loader.use_d435 = LaunchConfiguration('use_d435')
    description_loader.use_mock_components = LaunchConfiguration('use_mock_components')
    description_loader.gz_control_config_package = LaunchConfiguration('gz_control_config_package')
    description_loader.gz_control_config_file_path = LaunchConfiguration(
        'gz_control_config_file_path'
    )
    description_loader.manipulator_config_file_path = LaunchConfiguration(
        'manipulator_config_file_path'
    )
    description_loader.manipulator_links_file_path = LaunchConfiguration(
        'manipulator_links_file_path'
    )
    loaded_description = description_loader.load()

    moveit_config = (
        MoveItConfigsBuilder('crane_x7')
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    moveit_config.robot_description = {'robot_description': loaded_description}

    return LaunchDescription(
        [
            declare_port_name,
            declare_baudrate,
            declare_timeout_seconds,
            declare_manipulator_config_file_path,
            declare_manipulator_links_file_path,
            declare_use_gazebo,
            declare_use_d435,
            declare_use_mock_components,
            declare_gz_control_config_package,
            declare_gz_control_config_file_path,
            generate_move_group_launch(moveit_config),
            generate_moveit_rviz_launch(moveit_config),
            generate_static_virtual_joint_tfs_launch(moveit_config),
        ]
    )
