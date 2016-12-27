#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <sstream>
#include <stdlib.h>
#include <iostream>

ros::Publisher pub;
ros::Publisher pub2;
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){

	static tf::TransformBroadcaster br; 
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
	transform.setRotation(tf::Quaternion(
				msg->pose.pose.orientation.x,
				msg->pose.pose.orientation.y,
				msg->pose.pose.orientation.z,
				msg->pose.pose.orientation.w));
	ros::Time t = ros::Time::now();

	br.sendTransform(tf::StampedTransform(transform, t, "map", "odom"));

	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;

	float dist = (11.0F * x - 12.5F * y + 63.0F) / 16.651F;
	if(dist < 0.0) dist *= -1.0F;

	if(dist < 0.2){
		std_msgs::Empty empty_msg;
		pub.publish(empty_msg);
	}

	if( x > 3.8F && x < 5.0F && y > 7.0F){
		std_msgs::Empty empty_msg;
		pub2.publish(empty_msg);
	}
}


int main(int argc, char** argv){

	ros::init(argc, argv, "trans");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/base_pose_ground_truth", 10, &poseCallback);

	ros::NodeHandle n2;
	pub = n2.advertise<std_msgs::Empty>("/goal_seek", 10);

	ros::NodeHandle n3;
	pub2 = n3.advertise<std_msgs::Empty>("/almost_there", 10);
	ros::spin();
	return 0;

}


