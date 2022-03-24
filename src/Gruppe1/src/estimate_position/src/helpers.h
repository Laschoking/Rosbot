#pragma once

#include <ros/ros.h>
#include <string>

double getDoubleInput(const std::string& question, double def = 0);
bool getBoolInput(const std::string& question, bool def);
