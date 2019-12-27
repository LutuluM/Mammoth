#include <reverse_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(reverse_recovery::ReverseRecovery, nav_core::RecoveryBehavior)

namespace reverse_recovery
{
ReverseRecovery::ReverseRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void ReverseRecovery::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

ReverseRecovery::~ReverseRecovery()
{
  ;//delete world_model_;
}

void ReverseRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

	ROS_WARN("WARNING: PREFORMING REVERSE RECOVERY!");
	ros::Rate r(10);
	ros::NodeHandle n;
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	geometry_msgs::Twist cmd_vel;
	for(int i =0; i<10;i++){//runs for 1 sec
		cmd_vel.linear.x = -0.1;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z =0;
		vel_pub.publish(cmd_vel);
		r.sleep();
	}
	cmd_vel.linear.x = 0.0;
	vel_pub.publish(cmd_vel);
}

};

