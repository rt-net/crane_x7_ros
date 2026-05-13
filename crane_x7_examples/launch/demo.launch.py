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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_d435 = DeclareLaunchArgument(
        'use_d435', default_value='false', description='Use d435.'
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
    )

    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('crane_x7_control'), '/launch/crane_x7_control.launch.py']
        ),
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
            declare_use_d435,
            declare_rviz_config,
            declare_rviz_config_camera,
            move_group,
            control_node,
            realsense_node,
        ]
    )
