#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from std_msgs.msg import Int16, Float64

# サーボの情報を格納するグローバル変数
gripper_current = 0.0
gripper_dxl_position = 0
gripper_temp = 0.0

def current_callback(msg):
    global gripper_current
    gripper_current = msg.data

def dxl_position_callback(msg):
    global gripper_dxl_position
    gripper_dxl_position = msg.data

def temp_callback(msg):
    global gripper_temp
    gripper_temp = msg.data


def main():
    global gripper_current
    global gripper_dxl_position
    global gripper_temp

    FULL_OPEN  = 0.9
    FULL_CLOSE = 0.01

    # 電流のしきい値を設定
    CLOSE_THRESHOLD = 50
    OPEN_THRESHOLD  = -50

    rospy.init_node("joystick_example")
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    # グリッパーを動かすサーボモータの電流値を取得
    sub_gripper_current = rospy.Subscriber("/crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint/current",
            Float64, current_callback, queue_size=1)

    # グリッパーを動かすサーボモータの位置を取得
    sub_gripper_dxl_position = rospy.Subscriber("/crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint/dxl_position",
            Int16, dxl_position_callback, queue_size=1)

    # グリッパーを動かすサーボモータの温度を取得
    sub_gripper_temp = rospy.Subscriber("/crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint/temp",
            Float64, temp_callback, queue_size=1)


    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([FULL_OPEN, FULL_OPEN])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    gripper_is_open = True
    
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        print(" current [mA]: "     + str(gripper_current).ljust(6)
              + " dxl_position: "     + str(gripper_dxl_position).ljust(6)
              + " temp [deg C]: "     + str(gripper_temp).ljust(6))

        # 電流値がしきい値を超えれば、グリッパーを閉じる
        if gripper_is_open is True and gripper_current > CLOSE_THRESHOLD:
            gripper.set_joint_value_target([FULL_CLOSE, FULL_CLOSE])
            gripper.go()
            gripper_is_open = False

        # 電流値がしきい値を超えれば、グリッパーを開く
        if gripper_is_open is False and gripper_current < OPEN_THRESHOLD:
            gripper.set_joint_value_target([FULL_OPEN, FULL_OPEN])
            gripper.go()
            gripper_is_open = True

        r.sleep()

    print("shutdown")

    # SRDFに定義されている"vertical"の姿勢にする
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
