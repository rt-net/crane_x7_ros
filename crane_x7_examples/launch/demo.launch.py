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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_port_name = DeclareLaunchArgument(
        'port_name', default_value='/dev/ttyUSB0', description='Set port name.'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate', default_value='3000000', description='Set baudrate.'
    )

    declare_timeout_seconds = DeclareLaunchArgument(
        'timeout_seconds', default_value='1.0', description='Set timeout seconds.'
    )

    declare_use_d435 = DeclareLaunchArgument(
        'use_d435', default_value='false', description='Use d435.'
    )

    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo', default_value='false', description='Use gazebo or not.'
    )

    declare_use_mock_components = DeclareLaunchArgument(
        'use_mock_components', default_value='false', description='Use mock_components or not.'
    )

    config_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'), 'config', 'manipulator_config.yaml'
    )

    links_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'), 'config', 'manipulator_links.csv'
    )

    declare_manipulator_config_file_path = DeclareLaunchArgument(
        'manipulator_config_file_path',
        default_value=config_file_path,
        description='Set manipulator config file path.',
    )

    declare_manipulator_links_file_path = DeclareLaunchArgument(
        'manipulator_links_file_path',
        default_value=links_file_path,
        description='Set manipulator links file path.',
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

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=get_package_share_directory('crane_x7_moveit_config')
        + '/config/moveit.rviz',
        description='Set the path to rviz configuration file.',
        condition=UnlessCondition(LaunchConfiguration('use_d435')),
    )

    declare_rviz_config_camera = DeclareLaunchArgument(
        'rviz_config',
        default_value=get_package_share_directory('crane_x7_examples')
        + '/launch/camera_example.rviz',
        description='Set the path to rviz configuration file.',
        condition=IfCondition(LaunchConfiguration('use_d435')),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('crane_x7_moveit_config'),
                '/launch/run_move_group.launch.py',
            ]
        ),
        launch_arguments={
            'rviz_config': LaunchConfiguration('rviz_config'),
            'port_name': LaunchConfiguration('port_name'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout_seconds': LaunchConfiguration('timeout_seconds'),
            'manipulator_config_file_path': LaunchConfiguration('manipulator_config_file_path'),
            'manipulator_links_file_path': LaunchConfiguration('manipulator_links_file_path'),
            'use_gazebo': LaunchConfiguration('use_gazebo'),
            'use_d435': LaunchConfiguration('use_d435'),
            'use_mock_components': LaunchConfiguration('use_mock_components'),
            'gz_control_config_package': LaunchConfiguration('gz_control_config_package'),
            'gz_control_config_file_path': LaunchConfiguration('gz_control_config_file_path'),
        }.items(),
    )

    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('crane_x7_control'), '/launch/crane_x7_control.launch.py']
        ),
        launch_arguments={
            'port_name': LaunchConfiguration('port_name'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout_seconds': LaunchConfiguration('timeout_seconds'),
            'manipulator_config_file_path': LaunchConfiguration('manipulator_config_file_path'),
            'manipulator_links_file_path': LaunchConfiguration('manipulator_links_file_path'),
            'use_gazebo': LaunchConfiguration('use_gazebo'),
            'use_d435': LaunchConfiguration('use_d435'),
            'use_mock_components': LaunchConfiguration('use_mock_components'),
            'gz_control_config_package': LaunchConfiguration('gz_control_config_package'),
            'gz_control_config_file_path': LaunchConfiguration('gz_control_config_file_path'),
        }.items(),
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
        ),
        condition=IfCondition(LaunchConfiguration('use_d435')),
        launch_arguments={
            'camera_namespace': '',
            'device_type': 'd435',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items(),
    )

    return LaunchDescription(
        [
            declare_port_name,
            declare_baudrate,
            declare_timeout_seconds,
            declare_use_d435,
            declare_use_gazebo,
            declare_use_mock_components,
            declare_manipulator_config_file_path,
            declare_manipulator_links_file_path,
            declare_gz_control_config_package,
            declare_gz_control_config_file_path,
            declare_rviz_config,
            declare_rviz_config_camera,
            move_group,
            control_node,
            realsense_node,
        ]
    )
