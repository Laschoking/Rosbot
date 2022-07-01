#pragma once

#include <ros/ros.h>
#include <string>

// asks user for a double input
// def is the default, if only enter is pressed
double getDoubleInput(const std::string& question, double def = 0);

// asks user for a yes/no input
// def is the default, if only enter is pressed
bool getBoolInput(const std::string& question, bool def);
