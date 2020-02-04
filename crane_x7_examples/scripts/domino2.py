#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float64
import sys
import os

#def callback(self, data):
    #self.dep = rospy.Subscriber("depth", Float32, self.callback)

def main():
    rospy.init_node("crane_x7_pick_and_place_domino_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    
    #height = 0.155
    #width = 0.3

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    '''
    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    '''
    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    arm.set_named_target("home")
    arm.go()

    # 読み取り位置に配置
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.05
    target_pose.position.y = -0.3
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose) 
    arm.go()
    
    rospy.sleep(2.0) #少し待つ

#def callback(num):
    #rospy.Subscriber("depth", Float32, callback)
    #print(num)

    def callback(data):
        #rospy.Subscriber("depth", Float32, callback)
        print("ドミノの高さ")
        print(data)
        #data = float(data)
        #height = 0.3 - (data * 0.001)
        #width = 0.3
        width = 0.3

        if data > 0.118:
            print("4")
            height = 0.155
        elif 0.118 >= data > 0.096:
            print("3")
            height = 0.137
        elif 0.096 >= data > 0.077:
            print("2")
            height = 0.119
        elif 0.077 >= data > 0.062:
            print("1")
            height = 0.101
        else:
            print("実行不可")
        print(height)
            #print(height)
        #width = 0.3

    #ここからループ
        while height > 0.09: 
            
            # 掴む準備をする
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.0
            target_pose.position.y = -0.3
            target_pose.position.z = 0.2
            q = quaternion_from_euler(-3.13, 0.0, -3.12/2.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose) 
            arm.go()
            
            rospy.sleep(1.0)
        
            gripper.set_joint_value_target([1.0, 1.0])
            gripper.go()
        
            target_pose = geometry_msgs.msg.Pose() #掴みに行く
            target_pose.position.x = 0.0
            target_pose.position.y = -0.3
            target_pose.position.z = height
            q = quaternion_from_euler(-3.13, 0.0, -3.13/2.0)
            target_pose.orientation.x = q[0]        
            target_pose.orientation.y = q[1]      
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)
            arm.go()  # 実行
        
            rospy.sleep(1.0)
        
            gripper.set_joint_value_target([0.3, 0.3])
            gripper.go()# ハンドを閉じる
        
            # 持ち上げる
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.0
            target_pose.position.y = -0.3
            target_pose.position.z = 0.2
            q = quaternion_from_euler(-3.13, 0.0, -3.12/2.0)  # 上方から掴みに行く場合
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)  # 目標ポーズ設定
            arm.go()						  # 実行
            
            # 置きに行く
            # SRDFに定義されている"home"の姿勢にする
            arm.set_named_target("home")
            arm.go()

            # 下ろす
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = width
            target_pose.position.z = 0.10
            q = quaternion_from_euler(-3.4/2.0, -3.14, 0.0)  # 上方から掴みに行く場合
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)  # 目標ポーズ設定
            arm.go() 

            # 置く
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = width
            target_pose.position.z = 0.082
            q = quaternion_from_euler(-3.35/2.0, -3.14, 0.0) 
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)  # 目標ポーズ設定
            arm.go()  # 実行
        
            # ハンドを開く
            gripper.set_joint_value_target([0.6, 0.6])
            gripper.go()
            # 少し引く
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = width #- 0.05
            target_pose.position.z = 0.12
            q = quaternion_from_euler(-3.4/2.0, -3.14, 0.0)  # 上方から掴みに行く場合
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)  # 目標ポーズ設定
            arm.go()  # 実行

            # SRDFに定義されている"home"の姿勢にする
            arm.set_named_target("home")
            arm.go()

            height -= 0.018
            width -= 0.06
            print('h=' + str(height))

        else:
            # ループ終了後の処理(倒す)
            print("倒す") 
            gripper.set_joint_value_target([0.1, 0.1])
            gripper.go()

            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = width
            target_pose.position.z = 0.12
            q = quaternion_from_euler(-3.4/2.0, -3.14, 0.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)
            arm.go() 

            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = width + 0.1
            target_pose.position.z = 0.12
            q = quaternion_from_euler(-3.4/2.0, -3.14, 0.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm.set_pose_target(target_pose)
            arm.go()
    
    rospy.Subscriber("depth", Float64, callback)
    print("done")


if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
