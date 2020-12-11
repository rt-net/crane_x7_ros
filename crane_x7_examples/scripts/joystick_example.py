#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Vector3, Quaternion
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8


class JoyWrapper(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber("joy", Joy, self._callback_joy, queue_size=1)

        self._grip_updated = False
        self._pose_updated = False
        self._name_updated = False
        self._preset_updated = False
        self._do_shutdown = False # ジョイスティックによる終了操作フラグ

        self.TEACHING_NONE = 0
        self.TEACHING_SAVE = 1
        self.TEACHING_LOAD = 2
        self.TEACHING_DELETE = 3
        self._teaching_flag = self.TEACHING_NONE

        # /rane_x7_examples/launch/joystick_example.launch でキー割り当てを変更する
        self._BUTTON_SHUTDOWN_1 = rospy.get_param("~button_shutdown_1")
        self._BUTTON_SHUTDOWN_2 = rospy.get_param("~button_shutdown_2")

        self._BUTTON_NAME_ENABLE = rospy.get_param("~button_name_enable")
        self._BUTTON_NAME_HOME = rospy.get_param("~button_name_home")

        self._BUTTON_PRESET_ENABLE = rospy.get_param("~button_preset_enable")
        self._BUTTON_PRESET_NO1 = rospy.get_param("~button_preset_no1")

        self._BUTTON_TEACHING_ENABLE = rospy.get_param("~button_teaching_enable")
        self._BUTTON_TEACHING_SAVE = rospy.get_param("~button_teaching_save")
        self._BUTTON_TEACHING_LOAD = rospy.get_param("~button_teaching_load")
        self._BUTTON_TEACHING_DELETE = rospy.get_param("~button_teaching_delete")

        self._BUTTON_GRIP_ENABLE = rospy.get_param("~button_grip_enable")
        self._AXIS_GRIPPER = rospy.get_param("~axis_gripper")

        self._BUTTON_POSI_ENABLE = rospy.get_param("~button_posi_enable")
        self._BUTTON_RPY_ENABLE = rospy.get_param("~button_rpy_enable")
        self._AXIS_POSITION_X   = rospy.get_param("~axis_position_x")
        self._AXIS_POSITION_Y   = rospy.get_param("~axis_position_y")
        self._AXIS_POSITION_Z   = rospy.get_param("~axis_position_z")

        # 目標姿勢
        self._target_gripper_joint_values = [0.0, 0.0]
        self._target_arm_pose = Pose()
        self._target_arm_rpy = Vector3()
        self._target_name = "vertical"

        # ティーチング
        self._teaching_joint_values = []
        self._teaching_index = 0

    def set_target_gripper(self, joint_values):
        self._target_gripper_joint_values = joint_values

    def set_target_arm(self, pose):
        self._target_arm_pose = pose
        self._target_arm_rpy = self._orientation_to_rpy(pose.pose.orientation)

    def get_target_gripper(self):
        return self._target_gripper_joint_values

    def get_target_arm(self):
        return self._target_arm_pose

    def get_target_name(self):
        return self._target_name

    def do_shutdown(self):
        return self._do_shutdown

    def get_and_reset_grip_update_flag(self):
        self._grip_updated, flag = False, self._grip_updated
        return flag

    def get_and_reset_pose_update_flag(self):
        self._pose_updated, flag = False, self._pose_updated
        return flag

    def get_and_reset_name_update_flag(self):
        self._name_updated, flag = False, self._name_updated
        return flag

    def get_and_reset_preset_update_flag(self):
        self._preset_updated, flag = False, self._preset_updated
        return flag

    def get_and_reset_teaching_flag(self):
        self._teaching_flag, flag = self.TEACHING_NONE, self._teaching_flag
        return flag

    def save_joint_values(self, arm, gripper):
        # ティーチング
        # アームの各関節角度と、グリッパー開閉角度を配列に保存する
        rospy.loginfo("Teaching. Save")
        self._teaching_joint_values.append([arm, gripper])

    def load_joint_values(self):
        # ティーチング
        # 保存した角度情報を返す
        # 配列のインデックスが末尾まで来たら、インデックスを0に戻す
        # 何も保存していない場合はFalseを返す
        if self._teaching_joint_values:
            rospy.loginfo("Teaching. Load")
            joint_values = self._teaching_joint_values[self._teaching_index]
            self._teaching_index += 1

            if self._teaching_index >= len(self._teaching_joint_values):
                rospy.loginfo("Teaching. Index reset")
                self._teaching_index = 0
            return joint_values
        else:
            rospy.logwarn("Teaching. Joint Values is nothing")
            return False

    def delete_joint_values(self):
        # ティーチング
        # 角度情報が格納された配列を初期化する
        rospy.loginfo("Teaching. Delete")
        self._teaching_joint_values = []

    def _orientation_to_rpy(self, orientation):
        # クォータニオンをRPY(Role, Pitch, Yaw)に変換する
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        e = euler_from_quaternion((x, y, z, w))
        return Vector3(e[0], e[1], e[2])

    def _rpy_to_orientation(self, rpy):
        # RPY(Role, Pitch, Yaw)をクォータニオンに変換する
        q = quaternion_from_euler(rpy.x, rpy.y, rpy.z)
        return Quaternion(q[0], q[1], q[2], q[3])

    def _callback_joy(self, msg):
        # ジョイスティック信号用のコールバック関数
        # ボタン入力に合わせてフラグと制御量を設定する

        # シャットダウン
        if msg.buttons[self._BUTTON_SHUTDOWN_1] and msg.buttons[self._BUTTON_SHUTDOWN_2]:
            self._do_shutdown = True
            return

        # Group State
        if msg.buttons[self._BUTTON_NAME_ENABLE]:
            if msg.buttons[self._BUTTON_NAME_HOME]:
                self._target_name = "home"
                self._name_updated = True
                return 

        # PIDゲインプリセット
        if msg.buttons[self._BUTTON_PRESET_ENABLE]:
            if msg.buttons[self._BUTTON_PRESET_NO1]:
                self._preset_updated = True
                return

        # ティーチングフラグ
        if msg.buttons[self._BUTTON_TEACHING_ENABLE]:
            if msg.buttons[self._BUTTON_TEACHING_SAVE]:
                self._teaching_flag = self.TEACHING_SAVE
            elif msg.buttons[self._BUTTON_TEACHING_LOAD]:
                self._teaching_flag = self.TEACHING_LOAD
            elif msg.buttons[self._BUTTON_TEACHING_DELETE]:
                self._teaching_flag = self.TEACHING_DELETE
            return

        # グリッパー開閉
        if msg.buttons[self._BUTTON_GRIP_ENABLE]:
            # ジョイスティックを倒した量に合わせて、グリッパーを閉じる
            grip_value = 1.0 - math.fabs(msg.axes[self._AXIS_GRIPPER])

            if not grip_value:
                grip_value = 0.01

            self._target_gripper_joint_values = [grip_value, grip_value]
            self._grip_updated = True
            return
        
        # アーム姿勢(位置)変更
        if msg.buttons[self._BUTTON_POSI_ENABLE]:
            # ジョイスティックを倒した量に合わせて、目標姿勢を変える
            # ジョイスティックの仕様に合わせて、符号を変えてください
            self._target_arm_pose.pose.position.x -= msg.axes[self._AXIS_POSITION_X] * 0.1
            self._target_arm_pose.pose.position.y += msg.axes[self._AXIS_POSITION_Y] * 0.1
            self._target_arm_pose.pose.position.z += msg.axes[self._AXIS_POSITION_Z] * 0.1
            self._pose_updated = True

        # アーム姿勢(角度)変更
        if msg.buttons[self._BUTTON_RPY_ENABLE]:
            # ジョイスティックを倒した量に合わせて、目標姿勢を変える
            # ジョイスティックの仕様に合わせて、符号を変えてください
            self._target_arm_rpy.x -= msg.axes[self._AXIS_POSITION_X] * math.pi * 0.1
            self._target_arm_rpy.y += msg.axes[self._AXIS_POSITION_Y] * math.pi * 0.1
            self._target_arm_rpy.z += msg.axes[self._AXIS_POSITION_Z] * math.pi * 0.1
            self._target_arm_pose.pose.orientation = self._rpy_to_orientation(self._target_arm_rpy)
            self._pose_updated = True


def preset_pid_gain(pid_gain_no):
    # サーボモータのPIDゲインをプリセットする
    # プリセットの内容はcrane_x7_control/scripts/preset_reconfigure.pyに書かれている
    rospy.loginfo("PID Gain Preset. No." + str(pid_gain_no))
    preset_no = UInt8()
    preset_no.data = pid_gain_no
    pub_preset.publish(preset_no)
    rospy.sleep(1) # PIDゲインがセットされるまで待つ


def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    PRESET_DEFAULT  = 0
    PRESET_FREE     = 3
    pid_gain_no = PRESET_DEFAULT


    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    # 現在のアーム姿勢を、目標姿勢にセットする
    joy_wrapper.set_target_arm(arm.get_current_pose())
    joy_wrapper.set_target_gripper(gripper.get_current_joint_values())


    while joy_wrapper.do_shutdown() == False:
        # グリッパーの角度を変更する
        if joy_wrapper.get_and_reset_grip_update_flag():
            gripper.set_joint_value_target(joy_wrapper.get_target_gripper())
            gripper.go()

        # アームの姿勢を変更する
        if joy_wrapper.get_and_reset_pose_update_flag():
            arm.set_pose_target(joy_wrapper.get_target_arm())
            if arm.go() == False:
                # 現在のアーム姿勢を、目標姿勢にセットする
                joy_wrapper.set_target_arm(arm.get_current_pose())

        # アームの姿勢をpose_group (home or vertical)に変更する
        if joy_wrapper.get_and_reset_name_update_flag():
            arm.set_named_target(joy_wrapper.get_target_name())
            arm.go()
            # 現在のアーム姿勢を、目標姿勢にセットする
            joy_wrapper.set_target_arm(arm.get_current_pose())

        # アームのPIDゲインをプリセットする
        if joy_wrapper.get_and_reset_preset_update_flag():
            # PIDゲインを0:default, 1:Free に切り替える
            if pid_gain_no == PRESET_DEFAULT:
                pid_gain_no = PRESET_FREE
            else:
                pid_gain_no = PRESET_DEFAULT
                # PIDゲインをdefaultに戻すと、目標姿勢に向かって急に動き出す
                # 安全のため、現在のアームの姿勢を目標姿勢に変更する
                arm.set_pose_target(arm.get_current_pose())
                arm.go()
            preset_pid_gain(pid_gain_no)
            # 現在のアーム姿勢を、目標姿勢にセットする
            joy_wrapper.set_target_arm(arm.get_current_pose())

        # ティーチング
        teaching_flag = joy_wrapper.get_and_reset_teaching_flag()
        if teaching_flag == joy_wrapper.TEACHING_SAVE:
            # 現在のアーム姿勢、グリッパー角度を保存する
            # アームの角度が制御範囲内にない場合、例外が発生する
            try:
                arm_joint_values = arm.get_current_joint_values()
                gripper_joint_values = gripper.get_current_joint_values()

                arm.set_joint_value_target(arm_joint_values)
                gripper.set_joint_value_target(gripper_joint_values)
                joy_wrapper.save_joint_values(arm_joint_values, gripper_joint_values)
            except moveit_commander.exception.MoveItCommanderException:
                print("Error setting joint target. Is the target within bounds?")
        elif teaching_flag == joy_wrapper.TEACHING_LOAD:
            # 保存したアーム、グリッパー角度を取り出す
            joint_values = joy_wrapper.load_joint_values()
            if joint_values:
                arm.set_joint_value_target(joint_values[0])
                arm.go()
                gripper.set_joint_value_target(joint_values[1])
                gripper.go()
                # 現在のアーム姿勢を、目標姿勢にセットする
                joy_wrapper.set_target_arm(arm.get_current_pose())
                joy_wrapper.set_target_gripper(gripper.get_current_joint_values())
        elif teaching_flag == joy_wrapper.TEACHING_DELETE:
            # 保存したアーム姿勢、グリッパー角度を削除する
            joy_wrapper.delete_joint_values()


    rospy.loginfo("Shutdown...")

    # SRDFに定義されている"vertical"の姿勢にする
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("joystick_example")

    joy_wrapper = JoyWrapper()

    # PIDゲインプリセット用のPublisher
    pub_preset = rospy.Publisher("preset_gain_no", UInt8, queue_size=1)

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
