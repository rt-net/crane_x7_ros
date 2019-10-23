#include <ros/ros.h>
#include <x7_control/target.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <iostream>
#include <math.h>

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "target_param");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<x7_control::target>("target_param",1000);
  ros::Rate loop_rate(10);

  x7_control::target param;

  param.target_param = 0;
 int i = 0;

  while(ros::ok()){
          cout << "input:";
	  //cin >> param.target_param;
          if(i < 100){
		param.target_param = 30; 
	}
	  pub.publish(param);
	  ros::spinOnce();
  }
  return 0;
}
