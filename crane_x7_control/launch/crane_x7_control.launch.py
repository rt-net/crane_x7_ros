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
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
    description_loader.port_name = '/dev/ttyUSB0'
    description_loader.baudrate = '3000000'
    description_loader.timeout_seconds = '1.0'
    description_loader.manipulator_config_file_path = config_file_path
    description_loader.manipulator_links_file_path = links_file_path

    declare_loaded_description = DeclareLaunchArgument(
        'loaded_description',
        default_value=description_loader.load(),
        description='Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in crane_x7_description.'
    )

    crane_x7_controllers = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config',
        'crane_x7_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': LaunchConfiguration('loaded_description')},
                    crane_x7_controllers],
        output='screen',
        )

    spawn_joint_state_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_controller'],
                shell=True,
                output='screen',
            )

    spawn_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner crane_x7_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner crane_x7_gripper_controller'],
                shell=True,
                output='screen',
            )

    return LaunchDescription([
      declare_loaded_description,
      controller_manager,
      spawn_joint_state_controller,
      spawn_arm_controller,
      spawn_gripper_controller
    ])
