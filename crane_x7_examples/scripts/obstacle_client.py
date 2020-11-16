#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
import math
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler
from crane_x7_examples.srv import ObstacleAvoidance, ObstacleAvoidanceRequest


def euler_to_quaternion(role, pitch, yaw):
    q = quaternion_from_euler(role, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])

def main():
    rospy.init_node("obstacle_client")

    SERVICE_NAME = 'obstacle_avoidance_example/obstacle_avoidance'

    # サービスの起動を待つ
    try:
        rospy.wait_for_service(SERVICE_NAME, 10.0)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn('Service call failed: %s'%e)
        rospy.signal_shutdown('SERVICE CALL FAILED')
        return

    handler = rospy.ServiceProxy(SERVICE_NAME, ObstacleAvoidance)
    request = ObstacleAvoidanceRequest()

    # 目標姿勢を設定
    start_pose = Pose()
    start_pose.position.x = 0.35
    start_pose.position.y = -0.2
    start_pose.position.z = 0.1
    start_pose.orientation = euler_to_quaternion(0.0, math.pi/2.0, 0.0)

    goal_pose = copy.deepcopy(start_pose)
    goal_pose.position.y = 0.2
    
    # 障害物無しでアームを動かす
    request.start_pose = start_pose
    request.goal_pose = goal_pose
    request.obstacle_enable = False
    print('Request ...')
    result = handler(request)
    print(result)

    # 障害物を設定
    obstacle_name = "box"
    obstacle_size = Vector3(0.28, 0.16, 0.14)
    obstacle_pose_stamped = PoseStamped()
    obstacle_pose_stamped.header.frame_id = "/base_link"
    obstacle_pose_stamped.pose.position.x = 0.35
    obstacle_pose_stamped.pose.position.z = obstacle_size.z/2.0

    # 障害物有りでアームを動かす
    request.obstacle_enable = True
    request.obstacle_size = obstacle_size
    request.obstacle_pose_stamped = obstacle_pose_stamped
    request.obstacle_name = obstacle_name
    print('Request ...')
    result = handler(request)
    print(result)

    # 障害物の姿勢を変更
    obstacle_pose_stamped.pose.orientation = euler_to_quaternion(0.0, math.pi/2.0, math.pi/2.0)
    obstacle_pose_stamped.pose.position.z = obstacle_size.x/2.0

    # 障害物有りでアームを動かす
    request.obstacle_pose_stamped = obstacle_pose_stamped
    print('Request ...')
    result = handler(request)
    print(result)


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
