#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler

import copy
import math

def obstacle_callback(self, msg):
    print msg

def main():
    rospy.init_node("obstacle_avoidance_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)

    sub_obstacle = rospy.Subscriber("~obstacle", PoseStamped, obstacle_callback)

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1) # このsleepを消すと、sceneの更新ができなくなる

    # 障害物を取り除く
    scene.remove_world_object()

    # 目標姿勢の定義
    target_poses = []

    pose = Pose()
    pose.position.x = 0.3
    pose.position.y = -0.2
    pose.position.z = 0.1
    q = quaternion_from_euler(0.0, math.pi/2.0, 0.0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    target_poses.append(pose)

    pose2 = copy.deepcopy(pose)
    pose2.position.y = 0.2
    target_poses.append(pose2)

    # SRDFに定義されている"vertical"の姿勢にする
    arm.set_named_target("vertical")
    arm.go()
    arm.set_named_target("home")
    arm.go()

    target_no = 0
    while not rospy.is_shutdown():
        # 障害物が無い空間で、目標位置を行き来する
        arm.set_pose_target(target_poses[target_no])
        arm.go()

        target_no += 1
        if target_no >= 2:
            target_no = 0

    # # 障害物を追加する
    # box_size = (0.24, 0.16, 0.16)
    # box_pose = PoseStamped()
    # box_pose.header.frame_id = "/base_link"
    # box_pose.pose.position.x = 0.3
    # box_pose.pose.position.z = box_size[2] * 0.5
    # box_name = "box"
    # scene.add_box(box_name, box_pose, box_size)
    #
    # # 障害物を設置するための待ち時間
    # setting_time = 5.0
    # print "wait " + str(setting_time) + " seconds for setting an object."
    # rospy.sleep(setting_time)

    arm.set_named_target("vertical")
    arm.go()

    rospy.loginfo("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
