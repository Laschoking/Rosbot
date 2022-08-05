#pragma once

#include <ros/ros.h>
#include <string>
#include <sqlite3.h>

// asks user for a double input
// def is the default, if only enter is pressed
double getDoubleInput(const std::string& question, double def = 0);

// asks user for a yes/no input
// def is the default, if only enter is pressed
bool getBoolInput(const std::string& question, bool def);

void rotate_vehicle(double angle, double distance);

void rotate(ros::Publisher& velocity_pub, const double yaw,const double ang_vel, const double proc_yaw_mean = 0);
void drive(ros::Publisher& velocity_pub, const double distance, const double speed,const double proc_x_mean = 0);
double yaw_to_degree(double yaw);
double degree_to_yaw(double degree);
void resetImuOdom();
void resetEKF(ros::Publisher*  reset_ekf);
void resetAMCL(ros::Publisher*  reset_amcl);
