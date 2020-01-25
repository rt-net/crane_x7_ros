#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import rosnode

import time
import sys

def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("processing...")
    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    arm.set_named_target("home")
    arm.go()

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    def check_msg(data):

        print(" 加速度を表示：")
        print(data.linear_acceleration)

        if data.linear_acceleration.x <= 6.0:  #左に傾けた時

            gripper.go()
            pose = Pose()
            pose.position.x = 0.3
            pose.position.y = 0.2
            pose.position.z = 0.3
            arm.set_pose_target(pose)
            arm.go()
            sys.exit()
                        
        if data.linear_acceleration.x >= -6.0:   #右に傾けた時
            pose = Pose()
            pose.position.x = 0.3
            pose.position.y = -0.2
            pose.position.z = 0.3
            arm.set_pose_target(pose)
            arm.go()
            sys.exit()

        if data.linear_acceleration.y <= 6.0:  #上に傾けた時
            pose = Pose()
            pose.position.x = 0.3
            pose.position.y = 0.0
            pose.position.z = 0.5
            arm.set_pose_target(pose)
            arm.go()
            sys.exit()

        if data.linear_acceleration.y >= -6.0:   #下に傾けた時
            pose = Pose()
            pose.position.x = 0.3
            pose.position.y = 0.0
            pose.position.z = 0.2
            arm.set_pose_target(pose)
            arm.go()
            sys.exit()

    rospy.Subscriber('/imu/data_raw', Imu, check_msg)
    r = rospy.Rate(10)
    r.sleep()
    

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
