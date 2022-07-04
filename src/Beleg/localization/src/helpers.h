#pragma once

#include <ros/ros.h>
#include <string>

// asks user for a double input
// def is the default, if only enter is pressed
double getDoubleInput(const std::string& question, double def = 0);

// asks user for a yes/no input
// def is the default, if only enter is pressed
bool getBoolInput(const std::string& question, bool def);

void rotate_vehicle(double angle, double distance);

void rotate(ros::Publisher& velocity_pub, const double yaw);
void drive(ros::Publisher& velocity_pub, const double distance, const double speed);
double yaw_to_degree(double yaw);
double degree_to_yaw(double degree);