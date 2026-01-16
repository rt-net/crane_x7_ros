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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    crane_x7_controllers = os.path.join(
        get_package_share_directory('crane_x7_control'), 'config', 'crane_x7_controllers.yaml'
    )

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
    description_loader.manipulator_config_file_path = LaunchConfiguration('manipulator_config_file_path')
    description_loader.manipulator_links_file_path = LaunchConfiguration('manipulator_links_file_path')
    description_loader.use_gazebo = LaunchConfiguration('use_gazebo')
    description_loader.use_d435 = LaunchConfiguration('use_d435')
    description_loader.use_mock_components = LaunchConfiguration('use_mock_components')
    description_loader.gz_control_config_package = LaunchConfiguration('gz_control_config_package')
    description_loader.gz_control_config_file_path = LaunchConfiguration('gz_control_config_file_path')
    loaded_description = description_loader.load()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': loaded_description}],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[crane_x7_controllers],
    )

    spawn_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_controller'],
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['crane_x7_arm_controller'],
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['crane_x7_gripper_controller'],
    )

    return LaunchDescription([
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
        robot_state_publisher,
        controller_manager,
        spawn_joint_state_controller,
        spawn_arm_controller,
        spawn_gripper_controller
    ])
