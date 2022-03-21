#include <iostream>
#include <string>
#include <sstream>
#include <limits>
#include <chrono>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include "driver.h"

constexpr double distanceEpsilon = 0.01;
constexpr double angleEpsilon = 0.3;

constexpr double defaultDriveSpeed = 0.25;
constexpr double turnSpeed = 0.4;

geometry_msgs::Pose2D drive(ros::Publisher& velocity_pub, geometry_msgs::Pose2D currentPose, const double speed, const double duration, const double turn) {
   //std::cout << "Drive with speed " << speed << " and turn " << turn << std::endl;

   geometry_msgs::Twist velocity;

   velocity.linear.x = speed;
   velocity.linear.y = 0;
   velocity.linear.z = 0;

   velocity.angular.x = 0;
   velocity.angular.y = 0;
   velocity.angular.z = turn;

   velocity_pub.publish(velocity);

   geometry_msgs::Pose2D poseDelta;
   poseDelta.x = std::cos(currentPose.theta) * speed * duration;
   poseDelta.y = std::sin(currentPose.theta) * speed * duration;
   poseDelta.theta = turn * duration;

   return poseDelta;
}

void driveToPoint(ros::Publisher& velocity_pub, const double targetX, const double targetY, geometry_msgs::Pose2D& state) {
   double distanceToTarget = std::numeric_limits<double>::max();

   std::cout << "Driving to (" << targetX << ", " << targetY << ")" << std::endl;
   std::cout << "Current state: (" << state.x << ", " << state.y << ", " << state.theta << ")" << std::endl;
   
   const int REFRESHRATE = 20;
   ros::Rate loop_rate(REFRESHRATE);

   int counter = 0;

   double duration = 1. / REFRESHRATE;

   // loop until almost there
   while (distanceToTarget >= distanceEpsilon) {
      loop_rate.sleep();
      ros::spinOnce();

      // calculate where we need to go from current state
      auto dx = targetX - state.x;
      auto dy = targetY - state.y;

      auto rx = std::cos(state.theta);
      auto ry = std::sin(state.theta);

      // angle between the two vectors
      auto theta1 = std::atan2(dx, dy);
      auto theta2 = std::atan2(rx, ry);
      auto dtheta = theta2 - theta1;

      if (dtheta > M_PI) dtheta -= 2 * M_PI;

      //std::cout << "theta1: " << theta1 << " theta2: " << theta2 << " dtheta: " << dtheta << std::endl;

      double speed = 0;
      if (std::abs(dtheta) < angleEpsilon) {
         // only if the angle is roughly correct drive with a speed, otherwise turn on point
         speed = defaultDriveSpeed;
      }
      else {
         dtheta = (dtheta > 0) ? turnSpeed : -turnSpeed;
      }

      //std::cout << "dtheta: " << dtheta << std::endl;

      // send drive command
      auto dPose = drive(velocity_pub, state, speed, duration, dtheta);

      /**********************
       * KALMAN FILTER HERE *
       **********************/
      
      // TODO: need to get acml position and use the correct variances to use kalman filter
      state.x += dPose.x;
      state.y += dPose.y;
      state.theta += dPose.theta;
      
      // calculate new distance to target
      dx = targetX - state.x;
      dy = targetY - state.y;
      distanceToTarget = std::sqrt(dx * dx + dy * dy);
   }

   // stop driving for a second
   drive(velocity_pub, state, 0, duration, 0);

   std::this_thread::sleep_for(std::chrono::milliseconds(300));
}
