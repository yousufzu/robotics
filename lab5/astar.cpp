#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "unistd.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <utility>
#include <fstream>
#include <sstream>

ros::Publisher vel_pub;
std::pair<int, int> goal;
std::list<std::pair<int, int> > path;
std::vector<std::vector<bool> > isFree; //free or obstacle
std::vector<std::vector<double> > h;

void reCalculateH();
void runAStar();

//help taken from Wikipedia for conversion
double quatToYaw(double x, double y, double z, double w){ 
	double t0 = -2.0F * (y*y + z*z) + 1.0F;
	double t1 = 2.0F * (x*y - w*z);
	double deg = atan2(t1, t0);
	return deg; //in radians
	//return (deg*180.0F)/3.1416F;
}


void pubVelocity(double curX, double curY, double rot){

	//std::cout << "Publishing velocity, heading towards " << path.back().first << "," << path.back().second << std::endl;
	geometry_msgs::Twist msg;
	double xg = path.back().first; //goalx in global frame
	double yg = path.back().second; //goaly in global frame
	double xr = (xg-curX)*cos(rot) + (yg-curY)*sin(rot); //goalx in robot frame
	double yr = -1.0F*(xg-curX)*sin(rot) + (yg-curY)*cos(rot); //goaly in robot frame
	
	std::cout << "Goal in global frame: " << xg << ", " << yg << std::endl;
	std::cout << "Goal in robot frame: " << xr << ", " << yr << std::endl;

	double theta = atan2(yr, xr);
	//if(theta < 0.0F){
	//	theta = theta + 6.2832;
	//}
	msg.angular.z = theta;
	msg.linear.x = 4.0F; //0.5F * sqrt(pow(xr,2) + pow(yr,2));
	vel_pub.publish(msg);

}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){

	double x = msg->pose.pose.position.x + 9.0F;
	double y = msg->pose.pose.position.y + 9.8F;

	double rot = quatToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//std::cout << "Rotation of robot wrt global frame is: " << rot << std::endl;
	rot = fmod(rot, 2.0F * 3.14159);

	std::cout << "X: " << x << ". Y: " << y << ". Rotation: " << rot << std::endl;
	std::pair<int, int> next = path.back();

	double dist = sqrt(pow(x - next.first, 2) + pow(y - next.second, 2));
	if(dist < 1.0){
		std::cout << "Reached close to the next waypoint." << std::endl;
		path.pop_back();
	}

	pubVelocity(x, y, -1.0F*rot);

	double newx, newy;
	bool changed = false;

	if (ros::param::get("goalx",newx)){
		newx = newx + 9.0F;
		if(newx != goal.first){
			changed = true;
		}
	}
	if (ros::param::get("goaly",newy)){
		newy = newy + 9.8F;
		if(newy != goal.second){
			changed = true;
		}
	}

	if(changed){
		goal.first = newx;
		goal.second = newy;
		reCalculateH();
		runAStar();
	}

}

std::pair<int, int> findLowestF(std::vector<std::pair<int, int> >& openSet, std::vector<std::vector<double> >& f){

	int p = 0;
	int q = 0;

	double curMin = 111111111111111111;

	for(int i = 0; i < openSet.size(); i++){
		std::pair<int, int> node = openSet[i];
		if(f[node.first][node.second] < curMin){
			p = node.first;
			q = node.second;
			curMin = f[p][q];
		}

	}
	std::cout << "Lowest f-value: " << p << ", " << q << std::endl;
	std::pair<int,int> ret(p,q);
	return ret;

}

void runAStar(){
	
	std::vector<std::pair<int, int> > closedSet; //representing indices
	std::vector<std::pair<int, int> > openSet; //indices!!!
	std::pair<int, int> cur;
	cur.first = 1; //current x
	cur.second = 8; //current y
	openSet.push_back(cur);

	std::vector<std::vector<std::pair<int, int> > > cameFrom; //indices!!!!!
	cameFrom.resize(18);
	for(int i = 0; i < cameFrom.size(); i++){
		cameFrom[i].resize(20);
		for(int j = 0; j < cameFrom[i].size(); j++){
			std::pair<int, int> tmp(-1,-1);
			cameFrom[i][j] = tmp;
		}
	}

	std::vector<std::vector<double > > g; //cost from start till that node
	g.resize(18);
	for(int i = 0; i < g.size(); i++){
		g[i].resize(20);
		for(int j = 0; j < g[i].size(); j++){
			g[i][j] = 1000000; //million
		}
	}
	g[1][8] = 0;

	std::vector<std::vector<double> > f; //g + h
	f.resize(18);
	for(int i = 0; i < f.size(); i++){
		f[i].resize(20);
		for(int j = 0; j < f[i].size(); j++){
			f[i][j] = 1000000;
		}
	}
	f[1][8] = h[1][8];

	while(openSet.size() != 0){

		std::cout << "In while loop. OpenSet size: " << openSet.size() << std::endl;

		std::pair<int, int> current = std::pair<int,int>(findLowestF(openSet, f)); //use copy constructor to avoid memory issues
		if(current == goal){
			path.push_back(current);
			while(current.first != -1 && current.second != -1){
				std::cout << "Calculating final path." << std::endl;
				std::cout << "Passes through " << current.first << "," << current.second << std::endl;
				current = cameFrom[current.first][current.second];
				if(current.first != -1 && current.second != -1){
					path.push_back(current);
				}
			}
			return;
		}

		openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());
		closedSet.push_back(current);

		for(int i = 0; i < isFree.size(); i++){
			for(int j = 0; j < isFree[i].size(); j++){

				if(isFree[i][j] && ((abs(current.first-i) < 2 && current.second == j) || (abs(current.second-j) < 2 && current.first == i)) ){

					//std::cout << "Neighbor of current." << std::endl;
					std::pair<int, int> neighbor(i, j);

					if(std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()){
						//std::cout << "Neighbor was in closed set." << std::endl;
						continue;
					}

					double tent_g = g[current.first][current.second] + sqrt(pow(current.first - neighbor.first, 2) + pow(current.second - neighbor.second, 2));
					if(std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end()){ //does NOT contain
						openSet.push_back(neighbor);
					}
					
					else if(tent_g >= g[neighbor.first][neighbor.second]){
						//std::cout << "Not a better path." << std::endl;
						continue;
					}
					
					cameFrom[neighbor.first][neighbor.second] = current;
					g[neighbor.first][neighbor.second] = tent_g;
					f[neighbor.first][neighbor.second] = tent_g + h[neighbor.first][neighbor.second]; //g + h of neighbor

				}
				else{
					//std::cout << "Position " << i << "," << j << " was not a neighbor. isFree: " << isFree[i][j] << std::endl;
				}
			}
		}
	}
}

void reCalculateH(){
	
	for(int i = 0; i < h.size(); i++){
		for(int j = 0; j < h[i].size(); j++){
			h[i][j] = sqrt(pow(i-goal.first, 2) + pow(j-goal.second, 2));
		}
	}
}


int main(int argc, char** argv){

	ros::init(argc, argv, "astar");

	ros::NodeHandle n;
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	ros::Duration(2.0).sleep();
	goal.first = 13; // 4.5F + 9.0F
	goal.second = 19; // 9.0F + 9.8F;

	isFree.resize(18);
	for(int i = 0; i < isFree.size(); i++){
		isFree[i].resize(20);
	}

	int line_num = 1;
	std::ifstream ifile("../catkin_ws/src/lab5/src/map.txt");
	std::string line;
	std::cout << "About to read file." << std::endl;
	if(ifile.is_open()){
		while(std::getline(ifile, line)){
			std::cout << "Reading a line of the file." << std::endl;
			//20 - line is the y-coordinate
			int j = 20-line_num;
			for(int i = 0; i < 18; i++){
				isFree[i][j] = line.at(2*i + 7) == '0';
				std::cout << "isFree at " << i << ", " << j << ": " << isFree[i][j] << std::endl;
			}
			line_num++;
		}
		ifile.close();
	}
	else{
		std::cout << "Unable to open file." << std::endl;
	}

	h.resize(18);
	for(int i = 0; i < h.size(); i++){
		h[i].resize(20);
	}

	reCalculateH();
	std::cout << "Calculated h." << std::endl;
	runAStar();
	std::cout << "Ran A*." << std::endl;

	ros::NodeHandle n2;
	ros::Subscriber sub = n2.subscribe("/base_pose_ground_truth", 10, &poseCallback);

	//pubVelocity(-8.0 + 9.0F, -2.0F + 9.8F, 0.0F);
	//std::cout << "Published the 1st velocity." << std::endl;

	ros::spin();
	return 0;
}
