#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from std_msgs.msg import UInt8
import sys, tty, termios


class TeachingDataBase(object):
    def __init__(self):

        self._teaching_joint_values = []
        self._teaching_index = 0

    def save_joint_values(self, arm, gripper):
        # アームの各関節角度と、グリッパー開閉角度を配列に保存する
        self._teaching_joint_values.append([arm, gripper])

    def load_joint_values(self):
        # 保存した角度情報を返す
        # 配列のインデックスが末尾まで来たら、インデックスを0に戻す
        # 何も保存していない場合はFalseを返す
        if self._teaching_joint_values:
            joint_values = self._teaching_joint_values[self._teaching_index]
            self._teaching_index += 1

            if self._teaching_index >= len(self._teaching_joint_values):
                self._teaching_index = 0
            return joint_values
        else:
            rospy.logwarn("Joint Values is nothing")
            return False

    def load_all_joint_values(self):
        # 保存した角度情報をすべて返す
        # 何も保存していない場合はFalseを返す
        if self._teaching_joint_values:
            return self._teaching_joint_values
        else:
            rospy.logwarn("Joint Values is nothing")
            return False

    def delete_joint_values(self):
        # 角度情報が格納された配列を初期化する
        self._teaching_joint_values = []
        self._teaching_index = 0

    def get_current_index(self):
        return self._teaching_index

    def get_num_of_poses(self):
        return len(self._teaching_joint_values)


def getch():
    # 1文字のキーボード入力を返す
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


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
    gripper = moveit_commander.MoveGroupCommander("gripper")

    TORQUE_ON_PID = 0
    TORQUE_OFF_PID = 3

    CTRL_C = 3 # Unicode Code

    input_code = 0
    do_shutdown = False
    do_restart = True
    is_teaching_mode = True


    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    # preset_reconfigureノードが起動するまで待機
    rospy.sleep(1)

    # トルクをオフにする
    preset_pid_gain(TORQUE_OFF_PID)
    print "Torque OFF"

    print "TeachingExample"
    while do_shutdown is False:
        # リスタート時はキーボード入力の説明を表示
        if do_restart:
            poses_num = data_base.get_num_of_poses()
            pose_index = data_base.get_current_index() + 1 if poses_num else 0

            print ""
            if is_teaching_mode:
                print "[Teaching Mode] "
                print "[Next Pose:" + str(pose_index) + " of " + str(poses_num) + "]",
                print "[q]: Quit, [m]: switch to action Mode"
                print "[s]: Save, [d]: Delete"
            else:
                print "[Action Mode] "
                print "[Next Pose:" + str(pose_index) + " of " + str(poses_num) + "]",
                print "[q]: Quit, [m]: switch to teaching Mode"
                print "[l]: Load 1 pose, [a]: play All pose"

            print "Keyboard input >>>",
            do_restart = False
            
        # 文字入力
        input_key = getch()
        print input_key,
        input_code = ord(input_key)

        # シャットダウン
        if input_code == CTRL_C or input_code == ord('q') or input_code == ord('Q'):
            print "\nShutdown..."

            # トルクをONにしてから終了する
            if is_teaching_mode:
                print "Torque ON"
                # PIDゲインをdefaultに戻すと、目標姿勢に向かって急に動き出す
                # 安全のため、現在のアームの姿勢を目標姿勢に変更する
                rospy.sleep(1)
                arm.set_pose_target(arm.get_current_pose())
                arm.go()
                preset_pid_gain(TORQUE_ON_PID)

            do_shutdown = True
            continue

        # モード切替
        if input_code == ord('m') or input_code == ord('M'):
            print "\nMode switch"
            is_teaching_mode = not is_teaching_mode

            if is_teaching_mode:
                print "Torque OFF"
                preset_pid_gain(TORQUE_OFF_PID)
            else:
                print "Torque ON"
                # PIDゲインをdefaultに戻すと、目標姿勢に向かって急に動き出す
                # 安全のため、現在のアームの姿勢を目標姿勢に変更する
                rospy.sleep(1)
                arm.set_pose_target(arm.get_current_pose())
                arm.go()
                preset_pid_gain(TORQUE_ON_PID)

            do_restart = True
            continue


        if is_teaching_mode:
            # 現在のアーム姿勢、グリッパー角度を保存する
            if input_code == ord('s') or input_code == ord('S'):
                print "\nSave joint values"
                data_base.save_joint_values(
                        arm.get_current_joint_values(),
                        gripper.get_current_joint_values())
                do_restart = True
                continue

            # 保存したアーム姿勢、グリッパー角度を削除する
            if input_code == ord('d') or input_code == ord('D'):
                print "\nDelete joint values"
                data_base.delete_joint_values()
                do_restart = True
                continue

        else:
            # 保存したアーム、グリッパー角度を取り出す
            if input_code == ord('l') or input_code == ord('L'):
                print "\nLoad joint values"
                joint_values = data_base.load_joint_values()
                if joint_values:
                    arm.set_joint_value_target(joint_values[0])
                    arm.go()
                    gripper.set_joint_value_target(joint_values[1])
                    gripper.go()
                do_restart = True
                continue

            # 保存したアーム、グリッパー角度を連続再生する
            if input_code == ord('a') or input_code == ord('A'):
                print "\nplay All poses"
                all_joint_values = data_base.load_all_joint_values()
                if all_joint_values:
                    num_of_poses = len(all_joint_values)
                    for i, joint_values in enumerate(all_joint_values):
                        print "Play: " + str(i+1) + " of " + str(num_of_poses)
                        arm.set_joint_value_target(joint_values[0])
                        arm.go()
                        gripper.set_joint_value_target(joint_values[1])
                        gripper.go()
                do_restart = True
                continue



    # SRDFに定義されている"vertical"の姿勢にする
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("teaching_example")

    data_base = TeachingDataBase()

    # PIDゲインプリセット用のPublisher
    pub_preset = rospy.Publisher("preset_gain_no", UInt8, queue_size=1)

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
