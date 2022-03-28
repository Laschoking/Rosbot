#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// A 2D Pose together with variances
struct PoseWithVariances {
   geometry_msgs::Pose2D pose;
   // uninitialized variances are "infinite"
   double varx = std::numeric_limits<double>::max();
   double vary = std::numeric_limits<double>::max();
   double vartheta = std::numeric_limits<double>::max();
};

// filter predState and measuredState using kalman filter and return the result
geometry_msgs::Pose2D kalmanFilter(const geometry_msgs::Pose2D& currentState, const PoseWithVariances& predState, const PoseWithVariances& measuredState);

// drive one "tick" with a given speed. needs to get the duration of the tick, and how much to turn
// returns the ground truth delta that has been moved
geometry_msgs::Pose2D drive(ros::Publisher& velocity_pub, geometry_msgs::Pose2D currentPose, double speed, double duration, double turn);

// drive to a given point
void driveToPoint(ros::Publisher& velocity_pub, double targetX, double targetY, geometry_msgs::Pose2D& state);

// amcl callback function to be called when new amcl data is received
void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amclPose);
