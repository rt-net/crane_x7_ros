#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 RT Corporation
#
# Licensed under the RT Corporation NON-COMMERCIAL LICENSE.
# Please see https://github.com/rt-net/crane_x7_ros/blob/master/LICENSE
# for detail.

import rospy
import moveit_commander
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import rosnode
from tf.transformations import quaternion_from_euler

def follow_circular_path(
    center_position=Point(0.3, 0.0, 0.1), radius=0.1,
    num_of_waypoints=30, repeat=1,
    eef_step=0.01, jump_threshold=0.0, avoid_collisions=True):
    way_points = []
    q = quaternion_from_euler( 0.0, math.radians(180), 0.0 )
    target_orientation = Quaternion(q[0], q[1], q[2], q[3])

    for r in range(repeat):
        for i in range(num_of_waypoints):
            theta = 2.0 * math.pi * (i / float(num_of_waypoints))
            target_pose = Pose()
            target_pose.position.x = center_position.x + radius * math.cos(theta)
            target_pose.position.y = center_position.y + radius * math.sin(theta)
            target_pose.position.z = center_position.z
            target_pose.orientation = target_orientation
            way_points.append(target_pose)

    path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)
    arm.execute(path)

def main():
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    arm.set_named_target("home")
    arm.go()

    # 座標(x=0.3, y=0.0, z=0.1)を中心に、XY平面上に半径0.1 mの円を3回描くように手先を動かす
    follow_circular_path(center_position=Point(0.3, 0.0, 0.1), radius=0.1, repeat=3)

    arm.set_named_target("vertical")
    arm.go()

if __name__ == '__main__':
    rospy.init_node("cartesian_path_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
