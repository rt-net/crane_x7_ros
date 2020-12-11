#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import moveit_commander


def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    # 駆動速度を調整する
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(1.0)

    # SRDFに定義されている"vertical"の姿勢にする
    # すべてのジョイントの目標角度が0度になる
    arm.set_named_target("vertical")
    arm.go()

    # 目標角度と実際の角度を確認
    print("joint_value_target (radians):")
    print(arm.get_joint_value_target())
    print("current_joint_values (radians):")
    print(arm.get_current_joint_values())

    # 現在角度をベースに、目標角度を作成する
    target_joint_values = arm.get_current_joint_values()
    # 各ジョイントの角度を１つずつ変更する
    joint_angle = math.radians(-45)
    for i in range(7):
        target_joint_values[i] = joint_angle
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print(str(i) + "-> joint_value_target (degrees):"
              + str(math.degrees(arm.get_joint_value_target()[i]))
              + ", current_joint_values (degrees):"
              + str(math.degrees(arm.get_current_joint_values()[i]))
        )

    rospy.sleep(3)
    # 垂直に戻す
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("joint_values_example")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
