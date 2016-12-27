#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <sstream>
#include <stdlib.h>
#include <iostream>




int main(int argc, char** argv){

	ros::init(argc, argv, "pursuer");
	ros::NodeHandle node;
	ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 10);

	tf::TransformListener listener;
	ros::Rate rate(10.0);
	while(node.ok()){

		tf::StampedTransform transform;
		try{
			ros::Duration(1.0).sleep();
			ros::Time now = ros::Time::now();
			listener.lookupTransform("robot_1", "robot_0", now - ros::Duration(1.0), transform);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

    		geometry_msgs::Twist vel_msg;
    		vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    		vel_pub.publish(vel_msg);

    		rate.sleep();

	}

}
