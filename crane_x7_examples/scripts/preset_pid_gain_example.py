#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

from std_msgs.msg import UInt8

def preset_pid_gain(pid_gain_no):
    # サーボモータのPIDゲインをプリセットする
    # プリセットの内容はcrane_x7_control/scripts/preset_reconfigure.pyに書かれている
    rospy.loginfo("PID Gain Preset. No." + str(pid_gain_no))
    preset_no = UInt8()
    preset_no.data = pid_gain_no
    pub_preset.publish(preset_no)
    rospy.sleep(1) # PIDゲインがセットされるまで待つ


def main():
    rospy.init_node("preset_pid_gain_example")
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(1.0)

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    # サーボモータのPIDゲインをプリセット
    preset_pid_gain(0)

    # SRDFに定義されている"vertical"の姿勢にする
    arm.set_named_target("vertical")
    arm.go()
    arm.set_named_target("home")
    arm.go()

    # サーボモータのPIDゲインをプリセット
    # Pゲインが小さくなるので、X7を手で動かすことが可能
    preset_pid_gain(1)

    # X7を手で動かせることを確認するため、数秒間待つ
    sleep_seconds = 10
    for i in range(sleep_seconds):
        rospy.loginfo(str(sleep_seconds-i) + " counts left.")
        rospy.sleep(1)
        # 安全のため、現在のアームの姿勢を目標姿勢に変更する
        arm.set_pose_target(arm.get_current_pose())
        arm.go()

    # サーボモータのPIDゲインをプリセット
    preset_pid_gain(0)
    arm.set_named_target("home")
    arm.go()
    arm.set_named_target("vertical")
    arm.go()

    rospy.loginfo("done")


if __name__ == '__main__':
    # PIDゲインプリセット用のPublisher
    pub_preset = rospy.Publisher("preset_gain_no", UInt8, queue_size=1)

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
