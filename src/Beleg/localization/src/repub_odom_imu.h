#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg,ros::Publisher* odom_repub);
void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg, ros::Publisher* imu_repub);
