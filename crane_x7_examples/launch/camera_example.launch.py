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

from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declare_example_name = DeclareLaunchArgument(
        'example',
        default_value='color_detection',
        description=(
            'Set an example executable name: '
            '[color_detection, aruco_detection, point_cloud_detection]'
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=('Set true when using the gazebo simulator.'),
    )

    description_loader = RobotDescriptionLoader()

    moveit_config = MoveItConfigsBuilder('crane_x7').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    picking_node = Node(
        name='pick_and_place_tf',
        package='crane_x7_examples',
        executable='pick_and_place_tf',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    detection_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='crane_x7_examples',
        executable=LaunchConfiguration('example'),
        output='screen',
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            picking_node,
            detection_node,
            declare_example_name,
        ]
    )
