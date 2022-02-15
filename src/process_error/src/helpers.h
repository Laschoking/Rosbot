#pragma once

#include <ros/ros.h>
#include <string>

void drive(ros::Publisher& velocity_pub, const double distance, const double speed);
double getDoubleInput(const std::string& question, double def = 0);
bool getBoolInput(const std::string& question, bool def);
