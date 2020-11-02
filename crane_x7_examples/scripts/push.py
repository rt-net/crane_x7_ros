#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import math
import random
import geometry_msgs.msg
import rosnode
#import文を使ってパッケージ等を読み込み
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

gazebo_model_states = ModelStates()


def callback(msg):
    global gazebo_model_states
    gazebo_model_states = msg


def yaw_of(object_orientation):
    # クォータニオンをオイラー角に変換しyaw角度を返す
    euler = euler_from_quaternion(
        (object_orientation.x, object_orientation.y,
        object_orientation.z, object_orientation.w))

    return euler[2]


def main():
    global gazebo_model_states

    
    OK = True

    OBJECT_NAME = "wood_cube_5cm"   # 掴むオブジェクトの名前
    GRIPPER_OPEN = 1.2              # 掴む時のハンド開閉角度
    GRIPPER_CLOSE = 0.0          # 設置時のハンド開閉角度
    APPROACH_Z = 0.15               # 接近時のハンドの高さ
    LEAVE_Z = 0.20                  # 離れる時のハンドの高さ
    PICK_Z = 0.12                   # 掴む時のハンドの高さ
    PLACE_POSITIONS = [             # オブジェクトの設置位置 (ランダムに設置する)
            Point(0.4, 0.0, 0.0),
            Point(0.0, 0.3, 0.0),
            Point(0.0, -0.3, 0.0),
            Point(0.2, 0.2, 0.0),
            Point(0.2, -0.2, 0.0)]

    sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    rospy.sleep(1.0)

    while OK:
        
        # 押す準備
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.
        target_pose.position.z = 0.33
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # 押す
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.
        target_pose.position.z = 0.28
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
