#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "lab4/Motion.h"
#include "lab4/Observation.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include <iostream>
#include "stdlib.h"
#include "math.h"
#include <vector>

#define foreach BOOST_FOREACH

ros::Publisher pub;
ros::Publisher pub2;

visualization_msgs::Marker line;
visualization_msgs::Marker cube;
int my_id;

std::vector<std::vector<std::vector<double> > > P;
int curMaxX;
int curMaxY;
int curMaxT;

double c1;
double c2;
double e;

//help taken from Wikipedia for conversion
double quatToYaw(double x, double y, double z, double w){
	double t0 = -2.0F * (y*y + z*z) + 1.0F;
	double t1 = 2.0F * (x*y - w*z);
	double deg = atan2(t1, t0);
	return (deg*180.0F)/3.1416F;
}


void normalizeP(){

	double total = 0.0F;

	for(int i = 0; i < P.size(); i++){
		for(int j = 0; j < P[i].size(); j++){
			for(int k = 0; k < P[i][j].size(); k++){
				total += P[i][j][k];
			}
		}
	}

	for(int i = 0; i < P.size(); i++){
		for(int j = 0; j < P[i].size(); j++){
			for(int k = 0; k < P[i][j].size(); k++){
				P[i][j][k] = P[i][j][k] / total;
			}
		}
	}
}

//return indices of max probability in P
void updateMaxP(){

	std::cout << "Current max x,y,theta: " << curMaxX << ", " << curMaxY << ", " << curMaxT << std::endl;

	for(int i = 0; i < P.size(); i++){
		for(int j = 0; j < P[i].size(); j++){
			for(int k = 0; k < P[i][j].size(); k++){
				if(P[i][j][k] > P[curMaxX][curMaxY][curMaxT]){
					curMaxX = i;
					curMaxY = j;
					curMaxT = k;
				}
			}
		}
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "bayes");
	
	ros::NodeHandle n;
	pub = n.advertise<visualization_msgs::Marker>("vis", 10);
	ros::NodeHandle n2;
	pub2 = n2.advertise<visualization_msgs::Marker>("vis2", 10);

	line.header.frame_id = "/map";
	cube.header.frame_id = "/map";

	line.ns = "bayes0";
	cube.ns = "bayes1";

	line.id = my_id = 7;
	my_id++;
	cube.id = 0;

	line.type = visualization_msgs::Marker::LINE_STRIP;
	cube.type = visualization_msgs::Marker::CUBE;

	line.action = visualization_msgs::Marker::ADD;
	cube.action = visualization_msgs::Marker::ADD;

	line.scale.x = 0.1;
	cube.scale.x = 0.1;
	cube.scale.y = 0.1;
	cube.scale.z = 0.1;

	line.color.a = 1.0;
	line.color.b = 1.0;
	cube.color.r = 1.0;
	cube.color.g = 1.0;
	cube.color.a = 1.0;
	
	line.pose.orientation.w = 1.0;
	cube.pose.orientation.w = 1.0;

	//constants in Gaussian formula
	c1 = 0.05F;
	c2 = 3.0F;
	e = 2.7183;

	P.resize(35);
	for(int i = 0; i < P.size(); i++){
		P[i].resize(35);
		for(int j = 0; j < P[i].size(); j++){
			P[i][j].resize(36);
		}
	}
	//P is now a 35 x 35 x 36 matrix of doubles, all of which are 0

	for(int i = 0; i < P.size(); i++){
		for(int j = 0; j < P[i].size(); j++){
			for(int k = 0; k < P[i][j].size(); k++){
				P[i][j][k] = 0.1F;
			}
		}
	}
	P[12-1][28-1][21-1] = 1.0F;
	normalizeP();

	curMaxX = 11;
	curMaxY = 27;
	curMaxT = 20;

	std::vector<std::pair<double,double> > cubes; //ALL in meters!
	std::pair<double,double> tag0(1.25,5.25);
	std::pair<double,double> tag1(1.25,3.25);
	std::pair<double,double> tag2(1.25,1.25);
	std::pair<double,double> tag3(4.25,1.25);
	std::pair<double,double> tag4(4.25,3.25);
	std::pair<double,double> tag5(4.25,5.25);

	cubes.push_back(tag0);
	cubes.push_back(tag1);
	cubes.push_back(tag2);
	cubes.push_back(tag3);
	cubes.push_back(tag4);
	cubes.push_back(tag5);

	//publish the cubes
	ros::Duration(1.0).sleep();
	for(int i = 0; i < cubes.size(); i++){
		cube.id = i+1;
		cube.pose.position.x = cubes[i].first;
		cube.pose.position.y = cubes[i].second;
		cube.pose.position.z = 0;
		cube.header.stamp = ros::Time::now();
		pub2.publish(cube);
		//ros::Duration(3.0).sleep();
	}

	//publish the first point
	geometry_msgs::Point initP;
	initP.x = (curMaxX + 1) / 5;
	initP.y = (curMaxY + 1) / 5;
	line.points.push_back(initP);
	line.id = my_id++;
	line.header.stamp = ros::Time::now();
	pub.publish(line);

	//open the bag file
	rosbag::Bag bag;
	bag.open("../catkin_ws/src/lab4/grid.bag", rosbag::bagmode::Read);
	std::cout << "Rosbag file opened successfully." << std::endl;
	std::vector<std::string> topics;
	topics.push_back(std::string("Movements"));
	topics.push_back(std::string("Observations"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view){

		if (m.getTopic() == "Movements") {
			lab4::Motion::ConstPtr s = m.instantiate<lab4::Motion>();
			if (s != NULL){
				//std::cout << "Motion message. s->translation: " << s->translation << ". s->rotation1 in x,y,z,w: " << s->rotation1.x << ", " << s->rotation1.y << ", " << s->rotation1.z << ", " << s->rotation1.w << std::endl;
				double yaw1 = quatToYaw(s->rotation1.x, s->rotation1.y,s->rotation1.z,s->rotation1.w);
				if (yaw1 < 0) yaw1 += 360.0F;
				std::cout << "Rotation 1 in yaw: " << yaw1 << std::endl;
				double yaw2 = quatToYaw(s->rotation2.x, s->rotation2.y,s->rotation2.z,s->rotation2.w);
				if (yaw2 < 0) yaw2 += 360.0F;
				std::cout << "Rotation 2 in yaw: " << yaw2 << std::endl;
				double trans = 100.0F * s->translation; //meters to cm
				std::cout << "Translation: " << s->translation << std::endl;
				double theta = fmod(yaw1 + 10.0F*curMaxT, 360.0F); //in degrees
				double deltaX = trans * cos(theta*(3.1416F/180.0F));
				double deltaY = trans * sin(theta*(3.1416F/180.0F));

				int newX = curMaxX + deltaX/20; //20 cm for 1 square
				int newY = curMaxY + deltaY/20;
				int newT = ((int) fmod(theta + yaw2,360.0F)) / 10;

				std::cout << "Gaussian centered around: " << newX << ", " << newY << ", " << newT << std::endl;

				//create a Gaussian filter around newx,newy,newT
				
				for(int i = 0; i < P.size(); i++){
					for(int j = 0; j < P[i].size(); j++){
						for(int k = 0; k < P[i][j].size(); k++){
							double r2 = pow(newX - i, 2) + pow(newY - j, 2) + pow(newT - k, 2);
							double expo = (r2 * -1.0F) / c2;
							double newProb = c1 * pow(e, expo);
							//std::cout << "newProb at i,j,k " << i << "," << j << "," << k << ": " << newProb << std::endl;
							if(P[i][j][k] > 0.00001){ //values approach 0 otherwise!
								P[i][j][k] = P[i][j][k] * newProb;
							}
						}
					}
				}
				normalizeP();
				updateMaxP();
				geometry_msgs::Point p;
				p.x = ((double)curMaxX + 1.0F) / 5.0F; //cause it's an index and starts at 0. then convert to meters, *20 and divide by 100
				p.y = ((double)curMaxY + 1.0F) / 5.0F;
				line.points.push_back(p);
				line.id = my_id++;
				line.header.stamp = ros::Time::now();
				pub.publish(line);
				//ros::Duration(1.0).sleep();
			}
		}

		if (m.getTopic() == "Observations") {
			lab4::Observation::ConstPtr o = m.instantiate<lab4::Observation>();
			if (o != NULL){
				double range = o->range * 100.0F; //to cm
				double yaw = quatToYaw(o->bearing.x,o->bearing.y,o->bearing.z,o->bearing.w);
				double deltaX = range * cos((yaw*3.1416)/180.0F); //in cm
				double deltaY = range * sin((yaw*3.1416)/180.0F); //in cm
				
				int newX = curMaxX + deltaX/20; //20 cm for 1 square
				int newY = curMaxY + deltaY/20;
				int newT = curMaxT;

				std::cout << "Gaussian centered around: " << newX << ", " << newY << ", " << newT << std::endl;

				//create a Gaussian filter around newx,newy,newT
				
				for(int i = 0; i < P.size(); i++){
					for(int j = 0; j < P[i].size(); j++){
						for(int k = 0; k < P[i][j].size(); k++){
							double r2 = pow(newX - i, 2) + pow(newY - j, 2) + pow(newT - k, 2);
							double expo = (r2 * -1.0F) / c2;
							double newProb = c1 * pow(e, expo);
							//std::cout << "newProb at i,j,k " << i << "," << j << "," << k << ": " << newProb << std::endl;
							if(P[i][j][k] > 0.00001){ //values approach 0 otherwise!
								P[i][j][k] = P[i][j][k] * newProb;
							}
						}
					}
				}
				normalizeP();
				updateMaxP();
				geometry_msgs::Point p;
				p.x = ((double) curMaxX + 1.0F) / 5.0F; //cause it's an index and starts at 0. then convert to meters, *20 and divide by 100
				p.y = ((double) curMaxY + 1.0F) / 5.0F;
				line.points.push_back(p);
				line.id = my_id++;
				line.header.stamp = ros::Time::now();
				pub.publish(line);
			}
		}
	}
	bag.close();
	return 0;
}
