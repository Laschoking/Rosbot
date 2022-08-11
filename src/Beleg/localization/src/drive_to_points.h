#ifndef ROS_WS_DRIVE_TO_POINTS_H
#define ROS_WS_DRIVE_TO_POINTS_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sqlite3.h>
#include "monitor_process.h"


geometry_msgs::Point genPoint(double x, double y);
monitor_results* driveToPoint(geometry_msgs::Point* goal_point, ros::Publisher* move_base,sqlite3* db, int iteration, int goal_nr, const double speed, const double ang_vel);
void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg,ros::Publisher* odom_repub);
void ekfCallback(boost::shared_ptr< const nav_msgs::Odometry> ekf_msg);
void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amcl_msg);
void resAllSources(ros::Rate* loop_rate,ros::NodeHandle* n);
void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg,ros::Publisher* imu_repub);

#endif
