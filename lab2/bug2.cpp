#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "stdlib.h"
#include "math.h"
#include <sstream>
#include <stdlib.h>
#include <iostream>

ros::Publisher vel_pub;
//tf::TransformListener listener;
/*
   1 = GOAL_SEEK
   2 = TURN
   3 = WALL_FOLLOW
 */
int state;
float left_dis; //distance to keep
float cur_left; //current measurement
bool dont_change; //whether or not to change the state
void move(){

	geometry_msgs::Twist vel_msg;
	if(state == 1){

		//std::cout << "Current state: GOAL_SEEK." << std::endl;

		//Goal seek. Go directly towards the goal.
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try{
			//ros::Duration(1.0).sleep();
			ros::Time now = ros::Time::now();
			listener.waitForTransform("odom", "map", now, ros::Duration(2.0F));
			listener.lookupTransform("odom", "map", ros::Time(0), transform);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			return;
		}

		tf::Vector3 v(4.5F, 9.0F, 0.0F);
		tf::Vector3 v2 = transform(v);
		vel_msg.angular.z = 400.0 * atan2(v2.y(), v2.x());
		vel_msg.linear.x = 20.0F;
	}

	else if(state == 2){
		
		//Turn. turn right, until ranges[180] > 2.9.
		vel_msg.angular.z = -100.0F;
		vel_msg.linear.x = 0.05F;

	}

	else if(state == 3){

		//Wall follow.
		if(cur_left > left_dis + 0.1){
			vel_msg.angular.z = 1.0F;
			vel_msg.linear.x = 2.0F;
		}
		else if(cur_left < left_dis - 0.1){
			vel_msg.angular.z = -1.0F;
			vel_msg.linear.x = 2.0F;
		}
		else{
			vel_msg.angular.z = 0.0F;
			vel_msg.linear.x = 2.0F;
		}

	}
	vel_pub.publish(vel_msg);

}


void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){

	if(dont_change) return;

	if(state == 1 || state == 3){

		float range_mid = (scan->ranges[90] + 2.0*scan->ranges[180] + scan->ranges[270]) / 4.0F;
		if(range_mid < 1.5F){
			state = 2;
		}
	}
	else if(state == 2){
		if(scan->ranges[180] > 2.9){
			state = 3;
			left_dis = scan->ranges[359];
		}
	}

	cur_left = scan->ranges[359];
}

void goalSeekCallBack(const std_msgs::Empty::ConstPtr& empty_msg){

	if(state == 3){
		state = 1;
	}

}

void almostCallBack(const std_msgs::Empty::ConstPtr& empty_msg){
	state = 1;
	dont_change = true;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "bug2");
	ros::NodeHandle node;
	vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	ros::NodeHandle node2;
	ros::Subscriber sub = node2.subscribe("/base_scan", 10, &scanCallBack);

	ros::NodeHandle node3;
	ros::Subscriber sub2 = node3.subscribe("/goal_seek", 10, &goalSeekCallBack);

	ros::NodeHandle node4;
	ros::Subscriber sub3 = node4.subscribe("/almost_there", 10, &almostCallBack);

	state = 1;
	dont_change = false;
	ros::Rate rate(10.0);
	while(ros::ok()){
		move();
		ros::spinOnce();
	}

	return 0;
}
