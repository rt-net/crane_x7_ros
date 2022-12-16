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
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
from launch.substitutions import LaunchConfiguration

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    robot_description_semantic_config = load_file(
        'crane_x7_moveit_config', 'config/crane_x7.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('crane_x7_moveit_config', 'config/kinematics.yaml')

    declare_example_name = DeclareLaunchArgument(
        'example', default_value='pose_groupstate',
        description=('Set an example executable name: '
                     '[gripper_control, pose_groupstate, joint_values,'
                     'pick_and_place, cartesian_path]')
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    example_node = Node(name=["pick_and_place_tf", '_node'],
                        package='crane_x7_examples',
                        executable='pick_and_place_tf',
                        output='screen',
                        parameters=[{'robot_description': description_loader.load()},
                                    robot_description_semantic,
                                    kinematics_yaml])
    example_node2 = Node(name=[LaunchConfiguration('example'), '_node'],
                        package='crane_x7_examples',
                        executable=LaunchConfiguration('example'),
                        output='screen'
    )

    realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py']),
            launch_arguments={
                'device_type': 'd435',
                'pointcloud.enable': 'true',
            }.items()
        )

    # RViz
    rviz_config_file = get_package_share_directory(
        'crane_x7_moveit_config') + '/launch/camera_test.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        rviz_node,
        example_node,
        example_node2,
        realsense_node,
        declare_example_name
    ])
