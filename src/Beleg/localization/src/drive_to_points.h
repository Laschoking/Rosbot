#pragma once
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


geometry_msgs::Point genPoint(double x, double y);
void driveToPoint(geometry_msgs::Point* goal_point, ros::Publisher& move_base);
double getXOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point);
double getYawOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point);
void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amcl);
void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom);