#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

int rotate;
ros::Publisher cmd_out;
ros::Subscriber cmd_in, rotate_callback;

void subCallback(const std_msgs::Int8::ConstPtr msgs){
	rotate = msgs->data;
}

void subVelocity(const geometry_msgs::Twist::ConstPtr msgs){
	geometry_msgs::Twist mod = *msgs;
	//if(rotate)mod.linear.x = 0;
	if(mod.linear.x > .95)
		mod.linear.x = .95;
	if(mod.linear.x < -0.95)
		mod.linear.x = -.95;
	//if(rotate)mod.linear.x = 0;
	mod.angular.z *= 1.5;
	cmd_out.publish(mod);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Velocity Node");
	ros::NodeHandle nh;
	
	rotate = false;
	
	cmd_out = nh.advertise<geometry_msgs::Twist>("/yeti/actuation/control2",10);
	cmd_in = nh.subscribe("/yeti/actuation/control",10,subVelocity);
	rotate_callback = nh.subscribe("/yeti/auditory_feedback",1,subCallback);
	ros::spin();
}
