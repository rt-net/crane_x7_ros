#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler

import copy
import math

def obstacle_callback(self, msg):
    print msg

def main():
    rospy.init_node("obstacle_publisher")

    pub_obstacle = rospy.Publisher("obstacle_avoidance_example/obstacle", PoseStamped, queue_size=1)

    obstacle = PoseStamped()

    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():

        r.sleep()

    rospy.loginfo("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
