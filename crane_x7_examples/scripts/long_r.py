#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, Pose
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def check_msg(data):
    arm = moveit_commander.MoveGroupCommander("arm")
    #print(data)
    # q=data.orientation()
#    e = euler_from_quaternion((data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w))
#    return Vector3(e[0], e[1], e[2])
    pose=Pose()
#    pose.position.x=e[0]
#    pose.position.y=e[1]
#    pose.position.z=e[2]

#    pose.position.x=0.167
#    pose.position.y=0.0
#    pose.position.z=0.26r    
    
    arm_initial_pose = arm.get_current_pose().pose
#while not rospy.is_shutdown():
#重要なのはposition
    pose.position.x=-arm_initial_pose.position.x
#pose.position.x=0.0
    pose.position.y=arm_initial_pose.position.y
#pose.position.y=0.0
#pose.position.z=0.0
# pose.position.x=0.167
#    pose.position.y=0.20
#    pose.position.z=0.20    

    pose.position.z=arm_initial_pose.position.z

#    pose.orientation.x=0.0
#    pose.orientation.y=0.0
#    pose.orientation.z=0.0




    pose.orientation.x=data.orientation.x
    pose.orientation.y=data.orientation.y
    pose.orientation.z=data.orientation.z
    pose.orientation.w=data.orientation.w

#    pose.orientation.z=data.orientation.z
#    pose.orientation.w=1.0
    print(pose.orientation.x)
    print(data.orientation.x)
#pri    nt(i    )
#    for i in range(1000):
#        print(i)   
#    arm.set_num_planning_attempts(10)
    arm.set_max_acceleration_scaling_factor(0.1)
#    arm.set_max_velocity_scaling_factor(0.1)	
    arm.set_pose_target( pose )	# 目標ポーズ設定
    arm.get_goal_position_tolerance()
    arm.go()

#arm.shift_pose_target(x,y,z)
    print(arm_initial_pose)
#    arm.clear_pose_targets()
# pose.orientation.x=0
# pose.orientation.y=0
# pose.orientation.z=0.625
# pose.orientation.w=0
   # arm.set_pose_target( pose )	# 目標ポーズ設定
    #arm.go()							# 実行
    
    # 手動で姿勢を指定するには以下のように指定
    # target_pose = geometry_msgs.msg.Pose()
    # target_pose.position.x = 0.0
    # target_pose.position.y = 0.0
    # target_pose.position.z = 0.624
    # q = quaternion_from_euler( 0.0, 0.0, 0.0 )
    # target_pose.orientation.x = q[0]
    # target_pose.orientation.y = q[1]
    # target_pose.orientation.z = q[2]    # target_pose.orientation.w = q[3]
# max=50
#r = rospy.Rate(1.5) #()hz
# while True:
#for i in range(max):


def main():
    rospy.init_node("pose_groupstate_example")
    arm = moveit_commander.MoveGroupCommander("arm")
    robot = moveit_commander.RobotCommander()
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("processing...")
    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    arm.set_named_target("home")
    arm.go()

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # gripper.set_joint_value_target([0.01, 0.01])
    # gripper.go()
    


    # pub = rospy.Publisher('/sensor_msg/Imu', Quaternion, queue_size=10)
    # qua=Quaternion()


    rospy.Subscriber('/imu/data_raw',Imu,check_msg)
    r = rospy.Rate(0.5)
    r.sleep()
        # rospy.Subscriber('/imu/data_raw',Imu,check_msg)
        # sub=rospy.Subscriber('/imu/data_raw',Quaternion,check_msg)
        # rospy.Subscriber('Imu',Quaternion,check_msg)
        # rospy.Subscriber('/sensor_msg/Imu',Quaternion,check_msg)
    #    r.sleep()
        # 移動後の手先ポーズを表示
#        arm_goal_pose = arm.get_current_pose().pose
#        print("Arm goal pose:")
#        print(arm_goal_pose)
#        print("done")
#        print('processing...'+str(i+1)+'/'+str(max))
#
        # クォータニオンをRPY(Role, Pitch, Yaw)に変換する
        # x = qua.orientation.x
        # y = qua.orientation.y
        # z = qua.orientation.z
        # w = qua.orientation.w

        
        # q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        # target_pose.orientation.x = pub.orientation.x
        # target_pose.orientation.y = pub.orientation.y
        # r.sleep()
        # rospy.spin()


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
