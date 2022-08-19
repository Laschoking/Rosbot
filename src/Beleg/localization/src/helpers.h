#ifndef ROS_WS_HELPERS_H
#define ROS_WS_HELPERS_H
#include <ros/ros.h>
#include <string>
#include <thread>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>


// asks user for a double input
// def is the default, if only enter is pressed
double getDoubleInput(const std::string& question, double def = 0);

// asks user for a yes/no input
// def is the default, if only enter is pressed
bool getBoolInput(const std::string& question, bool def);


double yaw_to_degree(double yaw);
double degree_to_yaw(double degree);
void resetImuOdom();
void resetEKF(ros::Publisher*  reset_ekf);
void resetAMCL(ros::Publisher*  reset_amcl);
double getYawOffset(geometry_msgs::Pose* curr_pose, geometry_msgs::Point* goal_point);
double getXOffset(geometry_msgs::Pose* curr_pose, geometry_msgs::Point* goal_point);
int requestAmclUpdate();
double getYawDiff(geometry_msgs::Quaternion* or_1, geometry_msgs::Quaternion* or_2);

#endif