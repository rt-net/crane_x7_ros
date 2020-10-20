#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler


def main():    
    # はんこ
    seal_x = 0.20
    seal_y = -0.25

    seal_before_z = 0.20
    seal_z = 0.08
    seal_after_z = 0.20

    # 掴む
    grab_seal = 0.07

    inkpad_x = 0.20
    inkpad_y = -0.15
    inkpad_before_z = 0.20
    inkpad_z = 0.10

    a = math.pi/2.0
    b = 0
    c = 0
    
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    #はんこの手前まで
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = seal_x
    target_pose.position.y = seal_y
    target_pose.position.z = seal_before_z
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper.set_joint_value_target([0.2, 0.2])
    gripper.go()

    # はんこを掴む位置まで
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = seal_x
    target_pose.position.y = seal_y
    target_pose.position.z = seal_z
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
    rospy.sleep(1.0)

    gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()
    gripper.set_joint_value_target([0.09, 0.09])
    gripper.go()
    gripper.set_joint_value_target([0.08, 0.08])
    gripper.go()

    # はんこを掴む
    gripper.set_joint_value_target([grab_seal, grab_seal])
    gripper.go()
    rospy.sleep(1.0)

    # 持ち上げる
    print("持ち上げる")
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = seal_x
    target_pose.position.y = seal_y
    target_pose.position.z = seal_after_z
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
    rospy.sleep(1.0)

    # 朱肉まで移動
    print("移動")
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = inkpad_x
    target_pose.position.y = inkpad_y
    target_pose.position.z = inkpad_before_z
    q = quaternion_from_euler(a,b,c)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行
    rospy.sleep(1.0)

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
