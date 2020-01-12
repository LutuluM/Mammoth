#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cmath>
#include <vector>
//use a deque instead of a queue to insert at the front for adding points

typedef struct waypoint {geometry_msgs::Pose pose; double tolerance;} waypoint;

class map_class{
public:
	map_class(int);
private:
	bool init;
	int num_centroids;

	ros::NodeHandle nh;
	ros::Subscriber map_sub;
	std::vector<geometry_msgs::Point> centroids;
	std::vector<geometry_msgs::Point> foundPoints;
	std::vector<std::vector<geometry_msgs::Point>> pointsForAverage;
	
	double distance(geometry_msgs::Point,geometry_msgs::Point);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
};

map_class::map_class(int centroid_nums){
	num_centroids = centroid_nums;
	init = false;
	
	centroids.resize(centroid_nums);
	pointsForAverage.resize(centroid_nums);
	
	map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10, &map_class::mapCallback,this);
}

double map_class::distance(geometry_msgs::Point pt1, geometry_msgs::Point pt2){
	return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

void map_class::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgs){
	geometry_msgs::Point pt;
	int num;
	double dist;
	int rows = 300;
	int cols = 80;

	if(!foundPoints.empty())
		foundPoints.clear(); //clear points
	//get new points
	for(int x = 0; x< rows; x++) {
		for(int y = 0; y < cols; y++) {
			if((signed char) msgs->data[4000*(y+1960)+(x+1970)] > 1){//based on the default map size
				//array is mapped to platfield where [0,0] is (-3,-2)
				pt.x = x*0.05f - 3.0f;
				pt.y = y*0.05f - 2.0f;
				foundPoints.push_back(pt);
			}
		}
	}

	//init centroids
	if(!init){
		if(foundPoints.size() >= num_centroids){
			for(int i = 0; i< num_centroids ;i++){
				centroids[i].x = foundPoints[i].x;
				centroids[i].y = foundPoints[i].y;
			}
		init = true;
		}
	}

	//find nearest point
	if(init){
		num = 0;
		for(int i = 0; i< foundPoints.size() ;i++){
			pt = foundPoints[i];
			dist = distance(pt,centroids[0]);
			num = 0;
			for(int j = 1; j< num_centroids ;j++){
				if(dist> distance(pt,centroids[j]))
					num = j;
			}
			pointsForAverage[num].push_back(pt);
		}
		//calc new centroids
		pt.x = 0;
		pt.y = 0;
		for(int j = 0; j< num_centroids ;j++){
			for(int i = 0; i< pointsForAverage[j].size() ;i++){
				pt.x += pointsForAverage[j][i].x;
				pt.y += pointsForAverage[j][i].y;
			}
			centroids[j].x = pt.x / pointsForAverage[j].size();
			centroids[j].y = pt.y / pointsForAverage[j].size();
			pointsForAverage[j].clear();
			std::cout << "Centroid:" << j << " (" << centroids[j].x << ',' << centroids[j].y << ")\n";
		}
		std::cout << "--------------------------------------------\n";
	}

	/*
	//snowMap[x][y] = (signed char) msgs->data[4000*(y+1960)+(x+1970)];
	Map of the Matrix
	---------------------
	|x,y                |
	|                   |
	|-------------------|
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|-------------------|
	|                   |
	|       Start       |
	|                0,0|
	---------------------
	*/
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Map_node");
	map_class maps(2);
	ros::spin();
}

