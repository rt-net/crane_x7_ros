#!/usr/bin/env python
# coding: utf-8

import roslib
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import UInt8

class PRESET_RECONFIGURE:
    def __init__( self ):
        rospy.loginfo("Wait reconfig server...")
        self.joint_list = [
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_shoulder_fixed_part_pan_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_shoulder_revolute_part_tilt_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_upper_arm_revolute_part_twist_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_upper_arm_revolute_part_rotate_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_lower_arm_fixed_part_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_lower_arm_revolute_part_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_wrist_joint"}, \
                            {"control":"/crane_x7/crane_x7_control","joint":"crane_x7_gripper_finger_a_joint"}, \
                        ]

        ### プリセット定義 - 0.Initial parameters ###
        self.preset_init = []
        for i in range(len(self.joint_list)):
            self.preset_init.append( { "p_gain": 800, "i_gain": 0, "d_gain": 0 } )

        ### プリセット定義 - 1  脱力状態 ###
        self.preset_1 = [   
                            { "name":"crane_x7_shoulder_fixed_part_pan_joint",          "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_shoulder_revolute_part_tilt_joint",      "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_twist_joint",    "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_rotate_joint",   "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_fixed_part_joint",             "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_revolute_part_joint",          "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_wrist_joint",                            "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_gripper_finger_a_joint",                 "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                        ]

        ### プリセット定義 - 2 PゲインをInitial parametersの半分にする ###
        self.preset_2 = [   
                            { "name":"crane_x7_shoulder_fixed_part_pan_joint",          "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_shoulder_revolute_part_tilt_joint",      "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_twist_joint",    "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_rotate_joint",   "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_fixed_part_joint",             "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_revolute_part_joint",          "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_wrist_joint",                            "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_gripper_finger_a_joint",                 "p_gain": 400, "i_gain": 0, "d_gain": 0 },\
                        ]

        ### プリセット定義 - 3 ティーチングサンプル用のPIDゲイン ###
        self.preset_3 = [   
                            { "name":"crane_x7_shoulder_fixed_part_pan_joint",          "p_gain": 5, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_shoulder_revolute_part_tilt_joint",      "p_gain": 5, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_twist_joint",    "p_gain": 5, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_rotate_joint",   "p_gain": 5, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_fixed_part_joint",             "p_gain": 1, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_revolute_part_joint",          "p_gain": 1, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_wrist_joint",                            "p_gain": 1, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_gripper_finger_a_joint",                 "p_gain": 1, "i_gain": 0, "d_gain": 0 },\
                        ]

        ### プリセット定義 - 4 未設定。モノを掴んだ時や、アームの形状を変えた時に設定してみてください ###
        self.preset_4 = [   
                            { "name":"crane_x7_shoulder_fixed_part_pan_joint",          "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_shoulder_revolute_part_tilt_joint",      "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_twist_joint",    "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_upper_arm_revolute_part_rotate_joint",   "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_fixed_part_joint",             "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_lower_arm_revolute_part_joint",          "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_wrist_joint",                            "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"crane_x7_gripper_finger_a_joint",                 "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                        ]

        ### プリセットデータリスト ###
        self.preset_list = []
        self.preset_list.append( self.preset_init )
        self.preset_list.append( self.preset_1 )
        self.preset_list.append( self.preset_2 )
        self.preset_list.append( self.preset_3 )
        self.preset_list.append( self.preset_4 )

        self.reconfigure = []
        for joint in self.joint_list:
            self.reconfigure.append( {"client":dynamic_reconfigure.client.Client( joint["control"]+"/"+joint["joint"],timeout=10 ), \
                                      "joint:":joint["joint"]} )
        rospy.loginfo("Wait sub...")
        self.subscribe = rospy.Subscriber("preset_gain_no", UInt8, self.preset_no_callback)
        rospy.loginfo("Init finished.")


    def preset_no_callback(self, no):
        joint_no = 0
        for conf in self.reconfigure:
            conf["client"].update_configuration( {"position_p_gain":self.preset_list[no.data][joint_no]["p_gain"],"position_i_gain":self.preset_list[no.data][joint_no]["i_gain"],"position_d_gain":self.preset_list[no.data][joint_no]["d_gain"]} )
            joint_no = joint_no + 1


if __name__ == '__main__':
    rospy.init_node('preset_reconfigure')
    pr = PRESET_RECONFIGURE()
    rospy.spin()
