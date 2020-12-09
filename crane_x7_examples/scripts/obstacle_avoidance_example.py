#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from crane_x7_examples.srv import ObstacleAvoidance, ObstacleAvoidanceResponse

import copy
import math

server = None

def hook_shutdown():
    global server
    server.shutdown('rospy shutdown')

    # shutdown時にvertical姿勢へ移行する
    print('shutdown')
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    arm.set_named_target("vertical")
    arm.go()


def callback(req):
    SLEEP_TIME = 1.0 # sceneを更新するための待ち時間

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(SLEEP_TIME)

    # 障害物を取り除く
    scene.remove_world_object()
    rospy.sleep(SLEEP_TIME) 

    # 安全のため床を障害物として生成する
    floor_name = "floor"
    floor_size = (2.0, 2.0, 0.01)
    floor_pose = PoseStamped()
    floor_pose.header.frame_id = "/base_link"
    floor_pose.pose.position.z = -floor_size[2]/2.0
    scene.add_box(floor_name, floor_pose, floor_size)
    rospy.sleep(SLEEP_TIME)

    # 障害物を設置する
    if req.obstacle_enable:
        scene.add_box(req.obstacle_name, req.obstacle_pose_stamped, 
                (req.obstacle_size.x, req.obstacle_size.y, req.obstacle_size.z))
        rospy.sleep(SLEEP_TIME) 

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
    rospy.sleep(SLEEP_TIME) 

    return ObstacleAvoidanceResponse(result)


def main():
    global server

    rospy.init_node("obstacle_avoidance_example")
    rospy.on_shutdown(hook_shutdown)

    server = rospy.Service('~obstacle_avoidance', ObstacleAvoidance, callback)
    print('Ready to avoid obstacles')

    rospy.spin()


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
