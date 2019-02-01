#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import Joy


class JoyWrapper(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber("joy", Joy, self._callback_joy)

        self._updated = False # ジョイスティックの入力更新フラグ
        self._do_shutdown = False # ジョイスティックによる終了操作フラグ

        # /rane_x7_examples/launch/joystick_example.launch でキー割り当てを変更する
        self._BUTTON_SHUTDOWN_1 = rospy.get_param("~button_shutdown_1")
        self._BUTTON_SHUTDOWN_2 = rospy.get_param("~button_shutdown_2")
        self._BUTTON_ENABLE     = rospy.get_param("~button_enable")
        self._AXIS_POSITION_X   = rospy.get_param("~axis_position_x")
        self._AXIS_POSITION_Y   = rospy.get_param("~axis_position_y")
        self._AXIS_POSITION_Z   = rospy.get_param("~axis_position_z")


    def is_updated(self):
        return self._updated

    
    def reset_updated_flag(self):
        self._updated = False


    def do_shutdown(self):
        return self._do_shutdown


    def _callback_joy(self, msg):
        self._updated = True

        # shutdown1, 2を同時押しでexampleを終了する
        if msg.buttons[self._BUTTON_SHUTDOWN_1] and msg.buttons[self._BUTTON_SHUTDOWN_2]:
            self._do_shutdown = True


def main():
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()


    while joy_wrapper.do_shutdown() == False:
        if joy_wrapper.is_updated():
            rospy.loginfo("joy update!")
            joy_wrapper.reset_updated_flag()

    rospy.loginfo("Shutdown...")

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("joystick_example")

    joy_wrapper = JoyWrapper()


    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
