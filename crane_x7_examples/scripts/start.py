#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, Pose
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def check_msg(data):
    arm = moveit_commander.MoveGroupCommander("arm")
    pose=Pose()
    arm_initial_pose = arm.get_current_pose().pose

    pose.position.x=-arm_initial_pose.position.x    #重要なのはposition
    pose.position.y=arm_initial_pose.position.y
    pose.position.z=arm_initial_pose.position.z
  
    pose.orientation.x=data.orientation.x
    pose.orientation.y=data.orientation.y
    pose.orientation.z=data.orientation.z
    pose.orientation.w=data.orientation.w
  
    print(pose.orientation.x)
    print(data.orientation.x)
   
    arm.set_max_acceleration_scaling_factor(0.1)
    arm.set_pose_target( pose )	# 目標ポーズ設定
    arm.get_goal_position_tolerance()
    arm.go()
  
    print(arm_initial_pose)
    print(data.linear_acceleration)


def main():
    rospy.init_node("pose_groupstate_example")
    arm = moveit_commander.MoveGroupCommander("arm")
    robot = moveit_commander.RobotCommander()
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

    rospy.Subscriber('/imu/data_raw',Imu,check_msg)
    r = rospy.Rate(0.5)
    r.sleep()
     
if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
