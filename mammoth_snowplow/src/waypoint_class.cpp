#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <string>
#include <deque>
#include <iostream>
#include <fstream>
#include <vector>
//use a deque instead of a queue to insert at the front for adding points

typedef struct waypoint {geometry_msgs::Pose pose; double tolerance;} waypoint;

class waypoint_class{
	private:
		waypoint_class();//private null constructor
		int importWaypoints(std::string);
		void publishNextWaypoint();
		void checkPose();
		void addWaypointCallback(const geometry_msgs::Pose);
		void pathCallback(const nav_msgs::Path::ConstPtr&);
		void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr&);
		void printPose(geometry_msgs::Pose);
		void tfToPose(tf::StampedTransform,geometry_msgs::Pose&);

		std::deque<waypoint> waypoint_queue;//deque for pushing new waypoints to front of designated path
		ros::NodeHandle nh;
		ros::Publisher pose_pub,audio_pub;
		ros::Subscriber waypoint_sub, path_sub,status_sub;

		bool running,reached_waypoint;

		double tolerance_x,tolerance_y,tolerance_qz,tolerance_qw;
		double target_x,target_y,target_qz,target_qw;
		
		bool check;
		double lastchecked;
		signed char overshot;//0 none,1 forward, -1 backwards
		
	public:
		tf::TransformListener tfListener;
		waypoint_class(std::string);
		void addWaypoint(geometry_msgs::Pose);
		void run();
};

waypoint_class::waypoint_class(std::string filename){
	ros::NodeHandle privateNode("~");
	if(!privateNode.getParam("tolerance_x",tolerance_x)){
		tolerance_x = 0.2;
		std::cout << "MISSING PARAM:Defaulted tolerance_x to " << tolerance_x << '\n';
	}
	if(!privateNode.getParam("tolerance_y",tolerance_y)){
		tolerance_y = 0.2;
		std::cout << "MISSING PARAM:Defaulted tolerance_y to " << tolerance_y << '\n';
	}
	if(!privateNode.getParam("tolerance_qz",tolerance_qz)){
		tolerance_qz = 0.1;
		std::cout << "MISSING PARAM:Defaulted tolerance_qz to " << tolerance_qz << '\n';
	}
	if(!privateNode.getParam("tolerance_qw",tolerance_qw)){
		tolerance_qw = 0.1;
		std::cout << "MISSING PARAM:Defaulted tolerance_qw to " << tolerance_qw << '\n';
	}
	target_x = target_y = target_qz = target_qw = 0;

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
	audio_pub = nh.advertise<std_msgs::Int8>("/yeti/auditory_feedback",10);
	waypoint_sub = nh.subscribe<geometry_msgs::Pose>("/yeti/new_waypoints",10,&waypoint_class::addWaypointCallback,this);
	path_sub = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan",10, &waypoint_class::pathCallback,this);
	status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",10, &waypoint_class::statusCallback,this);

	bool running = false;
	reached_waypoint = false;
	overshot = 0;
	if(importWaypoints(filename)<0){
		std::cout << "Error not a valid file\n";
		exit(-1);
	}
	check = false;
	lastchecked = ros::Time::now().toSec();
}

int waypoint_class::importWaypoints(std::string filename){
	waypoint tempWaypoint;
	int position;
	std::string line,cell;
	tf::Quaternion quarter;
	double roll, pitch, yaw;

	std::ifstream waypointFile(filename, std::ifstream::in);
	std::getline(waypointFile,line); //remove the word line
	std::cout << line << '\n';
	if(waypointFile.is_open()){
		while(std::getline(waypointFile,line)){
			std::stringstream streamLine(line);
			position = 0;
			while(std::getline(streamLine,cell,',')){
				switch(position++){
					case 0:
						tempWaypoint.pose.position.x = std::stof(cell);
						break;
					case 1:
						tempWaypoint.pose.position.y = std::stof(cell);
						break;
					case 2:
						tempWaypoint.pose.position.z = std::stof(cell);
						break;
					case 3:
						roll = std::stof(cell);
						break;
					case 4:
						pitch = std::stof(cell);
						break;
					case 5:
						yaw = std::stof(cell);
						break;
					default:
						quarter.setRPY(roll,pitch,yaw);//could set roll and pitch to 0 since yeti should never be tilting
						tempWaypoint.pose.orientation.x = quarter.getX();
						tempWaypoint.pose.orientation.y = quarter.getY();
						tempWaypoint.pose.orientation.z = quarter.getZ();
						tempWaypoint.pose.orientation.w = quarter.getW();
						tempWaypoint.tolerance = std::stof(cell);
						waypoint_queue.push_back(tempWaypoint);
						break;
				}
			}
		}
		waypointFile.close();
	}
	else{
		std::cout << "Error opening file:" << filename << '\n';
		return -1;
	}
	if(!waypoint_queue.empty()){//print all waypoints to terminal
		running = true;
		geometry_msgs::Pose p;
		for(int loopcount = waypoint_queue.size();loopcount > 0;loopcount--){
			tempWaypoint = waypoint_queue.front();
			waypoint_queue.pop_front();
			printPose(tempWaypoint.pose);
			waypoint_queue.push_back(tempWaypoint);
		}
	}
	return 1;
}

void waypoint_class::publishNextWaypoint(){
	geometry_msgs::PoseStamped msgs;
	std::string cmdString;
	const char * cmd;
	static int waypoint_number = 0;

	msgs.header.frame_id = "map";
	msgs.header.stamp  = ros::Time(0);
	msgs.header.seq = waypoint_number++;
	msgs.pose.position = waypoint_queue.front().pose.position;
	msgs.pose.orientation = waypoint_queue.front().pose.orientation;
	pose_pub.publish(msgs);

	if(target_x<msgs.pose.position.x)//old(current pos) less than current
		overshot = 1;
	else
		overshot = -1;
	
	target_x = msgs.pose.position.x;
	target_y = msgs.pose.position.y;
	target_qz = msgs.pose.orientation.z;
	target_qw = msgs.pose.orientation.w;
	
	/*cmdString = "rosrun dynamic_reconfigure dyniparam set /move_base/DWAPlannerROS xy_goal_tolerance " + std::to_string(waypoint_queue.front().tolerance);//make sure that new waypoint will be published
	cmd = cmdString.c_str();
	system(cmd);*/
	tolerance_x = tolerance_y = waypoint_queue.front().tolerance + 0.1;//tolerance should be higher than planner tolerance
	
	std::cout << "----New Goal Set----\n";
	printPose(waypoint_queue.front().pose);
}

void waypoint_class::checkPose(){
	tf::StampedTransform transform;
	geometry_msgs::Pose p;
	std_msgs::Int8 msg;

	try{
		tfListener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("WAYPOINT_PUBLISHER: %s",ex.what());
		ros::Duration(1.0).sleep();
		return;
	}


	tfToPose(transform,p);//fixes negative w
	//std::cout << "\nCurrentTf/GoalTf\n";printPose(p);printPose(waypoint_queue.front().pose);

	if(std::abs( target_x - p.position.x) < tolerance_x)
		if(std::abs( target_y - p.position.y) < tolerance_y)
			if(std::abs( target_qz - p.orientation.z) < tolerance_qz)
				if(std::abs( target_qw - p.orientation.w) < tolerance_qw){
					reached_waypoint = true;
					check = false;
				}
				
	//check for oversooting
	switch(overshot){
		case 1: 
			if(target_x < p.position.x){
				reached_waypoint = true;
				check = false;
			}
			break;
		case -1:
			if(target_x > p.position.x){
				reached_waypoint = true;
				check = false;
			}
			break;
		default:break;
	}
	
	
	if(reached_waypoint){
		std::cout << "Reached Target Waypoint.\n";
		reached_waypoint = false;
		waypoint_queue.pop_front(); //pop moved here so that yeti will retain the original goal if a new goal is added from a callback
		if(waypoint_queue.empty()){
			running = false;
			std::cout << "All Waypoints Completed\n";
		}
		else{
			publishNextWaypoint();
			msg.data = 1;
			audio_pub.publish(msg);
		}
	}
}

void waypoint_class::addWaypoint(geometry_msgs::Pose msgs){
	waypoint tempWaypoint;
	tempWaypoint.pose = msgs;
	tempWaypoint.tolerance = tolerance_x;//default value for tolerance will be default waypoint tolerance
	waypoint_queue.push_front(tempWaypoint);
	this->publishNextWaypoint();
	//maybe some additional logic or rearrangement of this function
}

void waypoint_class::addWaypointCallback(const geometry_msgs::Pose msgs){
	addWaypoint(msgs);
}

void waypoint_class::pathCallback(const nav_msgs::Path::ConstPtr& msgs){
	target_x = msgs->poses.back().pose.position.x;
	target_y = msgs->poses.back().pose.position.y;
	target_qz = msgs->poses.back().pose.orientation.z;
	target_qw =msgs->poses.back().pose.orientation.w;
	if(target_qw<0){//reverse the negative because the tf messes up part of the 3rd quadrant
		target_qz *= -1;
		target_qw *= -1;
	}
}

void waypoint_class::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msgs){//publish next waypoint if waypoint publisher fails to send next one after goal is reached
	std_msgs::Int8 msg;

	if(msgs->status_list.size() > 0)
		if(msgs->status_list.back().status == 3){
			if(!check){
				check = true;
				lastchecked = ros::Time::now().toSec();
			}
			else if(lastchecked < ros::Time::now().toSec() - 5){
				lastchecked = ros::Time::now().toSec();
				waypoint_queue.pop_front();
				publishNextWaypoint();
				check = false;
				msg.data = 3;
				audio_pub.publish(msg);
			}
		}
}

void waypoint_class::printPose(geometry_msgs::Pose p){
	std::cout << "X:    Y:    Z:    QX:   QY:   QZ:   QW:   \n";
	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
}

void waypoint_class::tfToPose(tf::StampedTransform t,geometry_msgs::Pose& p){
	tf::pointTFToMsg(t.getOrigin(),p.position);
	tf::quaternionTFToMsg(t.getRotation(),p.orientation);
	if(p.orientation.w <0 ){//inverse negative w
		p.orientation.w *= -1;
		p.orientation.z *= -1;
	}
}

void waypoint_class::run(){
	ros::Rate r(10);//10hz
	ros::Duration(10).sleep();//Delay for Sub/Pub and Map to Initalize
	std_msgs::Int8 msg;
	
	this->publishNextWaypoint();
	msg.data = 1;
	audio_pub.publish(msg);
	while(ros::ok() && this->running){
		ros::spinOnce();
		this->checkPose();
		r.sleep();
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Waypoint_node");
	ros::NodeHandle privateNode("~");
	std::string filename;
	if(privateNode.getParam("filename",filename)){
		waypoint_class client(filename);
		//wait for the Map to be read before sending the waypoints
		while(ros::ok() && !client.tfListener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0)))
			std::cout << "WaypointGen: Waiting for /Map\n";
		client.run();
	}
	else{
		ROS_ERROR("Failed to get Param \'filename\' for waypoint file. %s",filename.c_str());
		exit(-1);
	}
}

