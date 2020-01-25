#!/usr/bin/env python
# coding: utf-8
#searching pose

import rospy
import time
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
import geometry_msgs.msg 
import moveit_commander
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion 
from control_msgs.msg import (      
    GripperCommandAction,
    GripperCommandGoal
 )
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy as np
from geometry_msgs.msg import Pose2D


class ArmJointTrajectoryExample(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)

        self.gripper_client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.gripper_goal = GripperCommandGoal()
        # Wait 5 Seconds for the gripper action server to start or exit
        self.gripper_client.wait_for_server(rospy.Duration(5.0))
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
         
    global sum_x
    sum_x = 0
    global sum_y
    sum_y = 0
    global count
    count = 0
    global x
    x = 0
    global y
    y = 0
    global z
    z = 0
           
    def callback(self, ms):
        #print("x座標 = {}".format(ms.x))
        #print("y座標 = {}".format(ms.y))
        #print("----------")

        global count
           
        if count < 100:
            global sum_x
            sum_x += ms.x
            global sum_y
            sum_y += ms.y
            count += 1

        else:
            print("ave_x :{}".format(sum_x/count))
            print("ave_y :{}".format(sum_y/count))
            self.calculate(sum_x/count,sum_y/count)
         
        #print("x座標 = {}".format(sum_x/count))
        #print("y座標 = {}".format(sum_y/count))
        print("count = {}".format(count))
        #print("x合計 = {}".format(sum_x))
        #print("y合計 = {}".format(sum_y))
        #print("----------")
       
    def calculate( self, ave_x, ave_y):
        res_x = z * tan(34.7) * ave_x / 320
        res_y = z * tan(21.25) * ave_y / 240
        print("calcurated\n res_x:{}".format(res_x))
        print("res_y:{}".format(res_y))
        self.pick( res_x, res_y)
        
    def search(self):
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

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()

        target_pose = geometry_msgs.msg.Pose()
        search_x = 0.176188627578
        search_y = -0.0182615322152
        search_z = 0.265958239801
        target_pose.position.x = search_x
        
        target_pose.position.y = search_y
        
        target_pose.position.z = search_z

        #掴む準備をする
        target_pose.orientation.x = -0.681381429346
        target_pose.orientation.y = -0.723920789866
        target_pose.orientation.z = 0.0649737640923
        target_pose.orientation.w = 0.0862348405328
        '''q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        '''
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()
        '''
        arm = moveit_commander.MoveGroupCommander("arm")                                                                                 
        arm.set_max_velocity_scaling_factor(0.4)
        gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
        gripper.wait_for_server()
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 4.0
        
        target_pose = Pose()

        search_x = 0.176188627578
        search_y = -0.0182615322152
        search_z = 0.265958239801
        target_pose.position.x = search_x
        
        target_pose.position.y = search_y
        
        target_pose.position.z = search_z
        
        '''
        '''
        target_pose.orientation.x = -0.681381429346
        target_pose.orientation.y = -0.723920789866
        target_pose.orientation.z = 0.0649737640923
        target_pose.orientation.w = 0.0862348405328
        arm.set_pose_target(target_pose)
        print("go mae\n")
        print(target_pose)
        arm.go()
        
        '''
        '''if arm.go() is False:
            print "Failed to approach an object."
            continue
        '''
        '''
        rospy.sleep(1.0)
        global x
        x = search_x
        global y
        y = search_y
        global z
        z = search_z
        '''
    def pick(self, res_x, res_y):
       
        arm = moveit_commander.MoveGroupCommander("arm")                                                                                 
        arm.set_max_velocity_scaling_factor(0.4)
        gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
        gripper.wait_for_server()
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 4.0
        
        tar_x = res_y + x
        tar_y = res_r + y

        target_pose = Pose()
        target_pose.position.x = tar_x
        target_pose.position.y = tar_y
        target_pose.position.z = 0.15
        q = quaternion_from_euler(math.pi, 0.0, math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        '''
        if arm.go() is False:
            print "Failed to approach an object."
            continue
        '''
        rospy.sleep(1.0)
        # 掴みに行く 
        target_pose.position.z = 0.12
        arm.set_pose_target(target_pose)
        '''
        if arm.go() is False:
            print "Failed to grip an object."
            continue
        '''
        rospy.sleep(1.0)
        gripper_goal.command.position = 0.6
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

        
    def go(self):
        #search position
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
     
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint", "crane_x7_shoulder_revolute_part_tilt_joint",
                                       "crane_x7_upper_arm_revolute_part_twist_joint", "crane_x7_upper_arm_revolute_part_rotate_joint",
                                       "crane_x7_lower_arm_fixed_part_joint", "crane_x7_lower_arm_revolute_part_joint",
                                       "crane_x7_wrist_joint",]
      
        joint_values = [0, 0, 0, -1.57, 0, -1.57, 1.57]
        print(joint_values) 
        position = math.radians(45.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=3.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)

        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))

         
    def feedback(self,msg):
        print("feedback callback")

if __name__ == "__main__":
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    arm_joint_trajectory_example.search()
