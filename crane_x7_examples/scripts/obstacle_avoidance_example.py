#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from crane_x7_examples.srv import ObstacleAvoidance, ObstacleAvoidanceResponse

import copy
import math


def hook_shutdown():
    # shutdown時にvertical姿勢へ移行する
    print 'shutdown'
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_named_target("vertical")
    arm.go()


def callback(req):
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    # 障害物を取り除く
    scene.remove_world_object()
    rospy.sleep(1) 

    # 障害物を設置する
    if req.obstacle_enable:
        scene.add_box(req.obstacle_name, req.obstacle_pose_stamped, 
                (req.obstacle_size.x, req.obstacle_size.y, req.obstacle_size.z))
        rospy.sleep(1) 

    result = True
    # home姿勢に移行
    arm.set_named_target('home')
    if arm.go() is False:
        result = False

    # スタート姿勢に移行
    arm.set_pose_target(req.start_pose)
    if arm.go() is False:
        result = False

    # ゴール姿勢に移行
    arm.set_pose_target(req.goal_pose)
    if arm.go() is False:
        result = False

    # home姿勢に移行
    arm.set_named_target('home')
    if arm.go() is False:
        result = False

    # 障害物を取り除く
    scene.remove_world_object()
    rospy.sleep(1) 

    return ObstacleAvoidanceResponse(result)


def main():
    rospy.init_node("obstacle_avoidance_example")
    rospy.on_shutdown(hook_shutdown)

    server = rospy.Service('~obstacle_avoidance', ObstacleAvoidance, callback)
    print 'Ready to avoid obstacles'

    rospy.spin()


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
