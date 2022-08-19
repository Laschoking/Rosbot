#ifndef ROS_WS_DRIVER_H
#define ROS_WS_DRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void rotate(ros::Publisher* velocity_pub, const double yaw,const double ang_vel, bool* new_amcl_data = NULL);
void drive(ros::Publisher* velocity_pub, const double distance, const double speed,bool* new_amcl_data = NULL);

#endif