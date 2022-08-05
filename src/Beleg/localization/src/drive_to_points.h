#ifndef ROS_WS_DRIVE_TO_POINTS_H
#define ROS_WS_DRIVE_TO_POINTS_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sqlite3.h>
#include "monitor_process.h"

geometry_msgs::Point genPoint(double x, double y);
monitor_results* driveToPoint(geometry_msgs::Point* goal_point, ros::Publisher& move_base,const double speed, const double ang_vel, double proc_x_mean, double proc_yaw_mean);
double getXOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point);
double getYawOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point);
void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg);
void ekfCallback(boost::shared_ptr< const nav_msgs::Odometry> ekf_msg);
void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amcl_msg);
void resAllSources(ros::Rate* loop_rate,ros::NodeHandle* n);

#endif
