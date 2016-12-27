#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdlib.h"
#include "math.h"
#include <iostream>

float range_mid = 3.0;
float cur_x = 2.0;
float cur_y = 0.0;
float cur_z = 0.0;

ros::Publisher vel_pub;

void velCallback(const sensor_msgs::LaserScan::ConstPtr& in_msg){
	range_mid = (in_msg->ranges[90] + in_msg->ranges[180] + in_msg->ranges[270]) / 3.0F;
	geometry_msgs::Twist msg;

	if(range_mid < 2.0){
		cur_x = 0.01F;
		cur_z = 1000.0F;
	}
	else{ //make it go straight if clear path
		cur_x = cur_x + 0.01F *(2.0F - cur_x);
		cur_z = 0.6F * cur_z;
	}
	msg.linear.x = cur_x;
	msg.linear.y = cur_y;
	msg.angular.z = cur_z;
	vel_pub.publish(msg);

}


int main(int argc, char** argv){
	ros::init(argc, argv, "evader");
	ros::NodeHandle n;
	vel_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);

	ros::NodeHandle n2;
	ros::Subscriber sub = n2.subscribe("/robot_0/base_scan", 10, &velCallback);
	ros::spin();
	return 0;
}
