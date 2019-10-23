#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <math.h>

#include"hardware.h"

double goal[JOINT_NUM] ={0,0,0,0,0,0,0,0};
double target[JOINT_NUM] ={0,0,0,0,0,0,0,0};
double ref[JOINT_NUM] ={0,0,0,0,0,0,0,0};
double def[JOINT_NUM] ={0,0,0,0,0,0,0,0};
double i[JOINT_NUM] ={0,0,0,0,0,0,0,0};

int main(int argc, char **argv){
  ros::init(argc, argv, "vis_joint_publisher");
  ros::NodeHandle nh;

  CR7 cr;
  if(!cr.Open_port())return 0;
  if(!cr.Set_port_baudrate())return 0;

  //publisher
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  cr.Enable_Dynamixel_Torque();

  ros::Rate loop_rate(10);
  ros::Duration dt;
  int count=0;
  while (ros::ok()){
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(8);
    js0.name[0] = "crane_x7_shoulder_fixed_part_pan_joint";
    js0.name[1] = "crane_x7_shoulder_revolute_part_tilt_joint";
    js0.name[2] = "crane_x7_upper_arm_revolute_part_twist_joint";
    js0.name[3] = "crane_x7_upper_arm_revolute_part_rotate_joint";
    js0.name[4] = "crane_x7_lower_arm_fixed_part_joint";
    js0.name[5] = "crane_x7_lower_arm_revolute_part_joint";
    js0.name[6] = "crane_x7_wrist_joint";
    js0.name[7] = "crane_x7_gripper_finger_a_joint";

    cr.Teaching_Play_Frame();

    js0.position.resize(8);
    js0.position[0]= (float)cr.joint_pose[0]*3.14/180;
    js0.position[1]= (float)cr.joint_pose[1]*3.14/180;
    js0.position[2]= (float)cr.joint_pose[2]*3.14/180;
    js0.position[3]= (float)cr.joint_pose[3]*3.14/180;
    js0.position[4]= (float)cr.joint_pose[4]*3.14/180;
    js0.position[5]= (float)cr.joint_pose[5]*3.14/180;
    js0.position[6]= (float)cr.joint_pose[6]*3.14/180;
    js0.position[7]= (float)cr.joint_pose[7]*3.14/180;
    joint_pub.publish(js0);

    def[5] = target[5] - (float)cr.joint_pose[5];
    i[5] += 10 * def[5]; 
    goal[5] = (def[5]*1.0) 
		+ (i[5]*0.01)
		+ ((def[5] - ref[5])/10)*0.1;

    ref[0] = (float)cr.joint_pose[0];
    ref[1] = (float)cr.joint_pose[1];
    ref[2] = (float)cr.joint_pose[2];
    ref[3] = (float)cr.joint_pose[3];
    ref[4] = (float)cr.joint_pose[4];
    ref[5] = def[5];
    ref[6] = (float)cr.joint_pose[6];
    ref[7] = (float)cr.joint_pose[7];

    cr.Move_Goal_Current( goal );
    count++;

    ros::spinOnce();
    loop_rate.sleep();
  } 
  cr.Disable_Dynamixel_Torque();
  return 0;
}
