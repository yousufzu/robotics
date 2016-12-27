#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "math.h"
#include "stdlib.h"
#include <iostream>

ros::Publisher pub;
ros::Publisher vel_pub;
visualization_msgs::Marker line;
int my_id;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

	geometry_msgs::Twist vel;
	vel.linear.x = 0.1F;
	vel.linear.y = 0.0F;
	vel.angular.z = 0.0F;
	vel_pub.publish(vel);

/*
	while: still some # of points in set
	- do ransac, get an answer: 2 pairs
	- find a set of inliers from the best fit
	- remove those points
*/

	std::vector<std::pair<float, float> > vec;

	for(int i = 0; i < 360; i++){
		int theta = ((float)i)/2.0 - 90.0F;
		theta = theta / 57.2958F; //radians
		float x = scan->ranges[i] * cos(theta);
		float y = scan->ranges[i] * sin(theta);
		std::pair<float, float> p(x, y);
		vec.push_back(p);
	}
	
	int num_it = 0; //number of iterations of while loop

	while(vec.size() > 100 && num_it < 10){

		std::pair<float, float> cur_p1;
		std::pair<float, float> cur_p2;
		int cur_inl = 0; //current # of inliers
		int max_inl = 0;

		for(int i = 0; i < 10; i++){
			//pick 2 random points
			int r1 = rand() % 360;
			int r2 = rand() % 360;
			if(r1 == r2){
				r2 = (r2 + 1) % 360;
			}
			float x1 = vec[r1].first;
			float y1 = vec[r1].second;
			float x2 = vec[r2].first;
			float y2 = vec[r2].second;

			float den = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
			float part_num = x2 * y1 - y2 * x1; //partial numerator
			//for ALL other points, check distance
			for(int j = 0; j < vec.size(); j++){
				float x3 = vec[j].first;
				float y3 = vec[j].second;
				float dist = (y2 - y1) * x3 - (x2 - x1) * y3 + part_num;
				if(dist < 0) dist *= -1.0F;
				dist = dist/den;

				//if within a threshold, # of inliers increases
				if(dist < 0.5){
					cur_inl++;
				}

			}
			//at the end: if cur_inl > max_inl, update cur_x cur_y and cur_inl
			if(cur_inl > max_inl){
				cur_p1 = vec[r1];
				cur_p2 = vec[r2];
				max_inl = cur_inl;
			}
			cur_inl = 0;
		}

		//remove the inliers for cur_p1 and cur_p2
		//do the whole thing all over again
		float x1 = cur_p1.first;
		float y1 = cur_p1.second;
		float x2 = cur_p2.first;
		float y2 = cur_p2.second;
		float den = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
		float part_num = x2 * y1 - y2 * x1; //partial numerator
		for(int j = 0; j < vec.size(); j++){

			float x3 = vec[j].first;
			float y3 = vec[j].second;
			float dist = (y2 - y1) * x3 - (x2 - x1) * y3 + part_num;
			if(dist < 0) dist *= -1.0F;
			dist = dist/den;

			if(dist < 0.5){
				vec.erase(vec.begin() + j);
				j--;
			}

		} //for-loop removing inliers

		//cur_p1 and cur_p2 define a line. Publish them
		geometry_msgs::Point p1;
		p1.x = cur_p1.first;
		p1.y = cur_p1.second;

		geometry_msgs::Point p2;
		p2.x = cur_p2.first;
		p2.y = cur_p2.second;

		line.id = my_id++;
		line.header.stamp = ros::Time::now();
		line.points.clear();
		line.points.push_back(p1);
		line.points.push_back(p2);
		pub.publish(line);

	} //while loop

}

int main(int argc, char** argv){
	ros::init(argc, argv, "ransac");

	ros::NodeHandle n2;
	pub = n2.advertise<visualization_msgs::Marker>("vis", 10);
	line.header.frame_id = "base_link";
	line.ns = "yz_vis";
	line.id = my_id = 0;
	my_id++;
	line.type = visualization_msgs::Marker::LINE_STRIP;
	line.action = visualization_msgs::Marker::ADD;
	line.scale.x = 0.1;
	line.color.b = 1.0;
	line.color.a = 1.0;

	ros::NodeHandle n3;
	vel_pub = n3.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("base_scan", 10, &scanCallback);
	ros::spin();
	return 0;
}
