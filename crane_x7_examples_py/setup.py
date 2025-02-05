from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'crane_x7_examples_py'

setup(
    name=package_name,
    version='5.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,
                      'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RT Corporation',
    maintainer_email='shop@rt-net.jp',
    description='python examples of CRANE-X7 ROS package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection = crane_x7_examples_py.aruco_detection:main',
            'color_detection = crane_x7_examples_py.color_detection:main',
            'gripper_control = crane_x7_examples_py.gripper_control:main',
            'joint_values = crane_x7_examples_py.joint_values:main',
            'pick_and_place_tf = crane_x7_examples_py.pick_and_place_tf:main',
            'pick_and_place = crane_x7_examples_py.pick_and_place:main',
            'pose_groupstate = crane_x7_examples_py.pose_groupstate:main',
        ],
    },
)
