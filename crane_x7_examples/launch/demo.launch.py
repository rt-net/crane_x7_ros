# Copyright 2022 RT Corporation
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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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

    declare_use_d435 = DeclareLaunchArgument(
        'use_d435',
        default_value='false',
        description='Use d435.'
    )

    config_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config',
        'manipulator_config.yaml'
    )

    links_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config',
        'manipulator_links.csv'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description_loader.baudrate = LaunchConfiguration('baudrate')
    description_loader.use_d435 = LaunchConfiguration('use_d435')
    description_loader.timeout_seconds = '1.0'
    description_loader.manipulator_config_file_path = config_file_path
    description_loader.manipulator_links_file_path = links_file_path

    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_x7_moveit_config'),
                '/launch/run_move_group.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('use_d435')),
            launch_arguments={
                'loaded_description': description
            }.items()
        )

    rviz_config_file = get_package_share_directory(
        'crane_x7_examples') + '/launch/camera_example.rviz'
    move_group_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_x7_moveit_config'),
                '/launch/run_move_group.launch.py']),
            condition=IfCondition(LaunchConfiguration('use_d435')),
            launch_arguments={
                'loaded_description': description,
                'rviz_config_file': rviz_config_file
            }.items()
        )

    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_x7_control'),
                '/launch/crane_x7_control.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py']),
            condition=IfCondition(LaunchConfiguration('use_d435')),
            launch_arguments={
                'device_type': 'd435',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
            }.items()
        )

    return LaunchDescription([
        declare_port_name,
        declare_baudrate,
        declare_use_d435,
        move_group,
        move_group_camera,
        control_node,
        realsense_node
    ])
