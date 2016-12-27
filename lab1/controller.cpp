#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <sstream>
#include <stdlib.h>
#include <iostream>

void poseCallback0(const nav_msgs::Odometry::ConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
	transform.setRotation(tf::Quaternion(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_0"));

}

void poseCallback1(const nav_msgs::Odometry::ConstPtr& msg){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
        transform.setRotation(tf::Quaternion(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_1"));

}




int main(int argc, char** argv){

	ros::init(argc, argv, "controller");

	ros::NodeHandle n2;
	ros::NodeHandle n3;

	ros::Subscriber sub = n2.subscribe("/robot_0/base_pose_ground_truth", 10, &poseCallback0);
	ros::Subscriber sub2 = n3.subscribe("/robot_1/base_pose_ground_truth", 10, &poseCallback1);
	ros::spin();
	return 0;

}
