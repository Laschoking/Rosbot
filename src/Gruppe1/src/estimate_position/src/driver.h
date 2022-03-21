#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// drive one "tick" with a given speed. needs to get the duration of the tick, and how much to turn
// returns the ground truth delta that has been moved
geometry_msgs::Pose2D drive(ros::Publisher& velocity_pub, geometry_msgs::Pose2D currentPose, double speed, double duration, double turn);

// drive to a given point
void driveToPoint(ros::Publisher& velocity_pub, double targetX, double targetY, geometry_msgs::Pose2D& state);
