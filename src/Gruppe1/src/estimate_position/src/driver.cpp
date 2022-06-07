#include <iostream>
#include <string>
#include <sstream>
#include <limits>
#include <chrono>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include "driver.h"

// process variances (x and y in robot coordinates)
/* Previous calculated by curve fitting
constexpr double ProcessVarX = 0.00576893;
constexpr double ProcessVarY = 0.01463654;
*/
constexpr double ProcessVarX = 6.6530407057e-6;
constexpr double ProcessVarY = 3.23939701222e-5;
constexpr double ProcessVarTheta = 1;
//constexpr double ProcessVarX = 0.5;
//constexpr double ProcessVarY = 0.5;

// measure variances (x and y in map coordinates)
/* Previous calculated by curve fiting
constexpr double MeasureVarX = 0.6645;
constexpr double MeasureVarY = 0.15095287;
*/
constexpr double MeasureVarX = 0.000265293475;
constexpr double MeasureVarY = 0.001156407;
//constexpr double MeasureVarX = 0.1;
//constexpr double MeasureVarY = 0.1;
constexpr double MeasureVarTheta = 1;

// the maximum allowed distance from target before it is considered reached
constexpr double distanceEpsilon = 0.03;
// if the angle between robot direction vector and robot to target vector is greater than this, turn in place
constexpr double angleEpsilon = 0.5;

// the update rate for the robot
constexpr int REFRESHRATE = 20;

// some named constants for robot move speeds
constexpr double maxDriveSpeed = 0.25;
constexpr double minDriveSpeed = 0.05;
constexpr double turnSpeed = 0.3;

// will become true if amcl has published new data
bool newAmclData = false;
// the last published amcl pose
geometry_msgs::Pose currentAmclData;

// normalizes the given angle to -PI to PI
inline double normalizeAngle(double theta) {
   if (theta > M_PI) theta -= 2 * M_PI;
   if (theta < -M_PI) theta += 2 * M_PI;
   return theta;
}

geometry_msgs::Pose2D kalmanFilter(const geometry_msgs::Pose2D& currentState, const PoseWithVariances& predState, const PoseWithVariances& measuredState) {
   geometry_msgs::Pose2D state;

   // "rotate" the variances of the predState by theta
   double predRotVarx = std::abs(std::cos(currentState.theta) * predState.varx) + std::abs(std::cos(currentState.theta + M_PI / 2.) * predState.vary);
   double predRotVary = std::abs(std::sin(currentState.theta) * predState.varx) + std::abs(std::sin(currentState.theta + M_PI / 2.) * predState.vary);

   // calculate the ks for each dimension
   double kx = predRotVarx / (predRotVarx + measuredState.varx);
   double ky = predRotVary / (predRotVary + measuredState.vary);
   double ktheta = predState.vartheta / (predState.vartheta + measuredState.vartheta);

   // unify thetas if they are more than 180 degrees apart
   if (std::abs(predState.pose.theta - measuredState.pose.theta) > M_PI) {
      // wrap the negative one around to positive so the kalman filter doesn't give strange results
      // exactly one of them has to be negative
      if (predState.pose.theta < 0) predState.pose.theta + (2 * M_PI);
      if (measuredState.pose.theta < 0) measuredState.pose.theta + (2 * M_PI);
   }

   // apply kalman filter to each dimension
   state.x = predState.pose.x + (kx * (measuredState.pose.x - predState.pose.x));
   state.y = predState.pose.y + (ky * (measuredState.pose.y - predState.pose.y));
   state.theta = normalizeAngle(predState.pose.theta + (ktheta * (measuredState.pose.theta - predState.pose.theta)));

   return state;
}

geometry_msgs::Pose2D drive(ros::Publisher& velocity_pub, geometry_msgs::Pose2D currentPose, const double speed, const double duration, const double turn) {
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
   
   ros::Rate loop_rate(REFRESHRATE);

   int counter = 0;

   double duration = 1. / REFRESHRATE;

   // loop until almost there
   while (distanceToTarget >= distanceEpsilon) {
      loop_rate.sleep();
      ros::spinOnce();

      // current trajectory vector
      auto rx = std::cos(state.theta);
      auto ry = std::sin(state.theta);

      // calculate where we need to go from current state
      auto dx = targetX - state.x;
      auto dy = targetY - state.y;

      // calculate angles of the two vectors with the angles from [0, 2PI]
      auto targetTheta = std::atan2(dx, dy);
      auto stateTheta = std::atan2(rx, ry);
      if (targetTheta < 0) targetTheta += 2 * M_PI;
      if (stateTheta < 0) stateTheta += 2 * M_PI;
      
      // get the signed angle difference
      auto dtheta = -(std::fmod((targetTheta - stateTheta + 3 * M_PI), 2 * M_PI) - M_PI);

      double speed = 0;
      if (std::abs(dtheta) < angleEpsilon) {
         // only if the angle is roughly correct drive with a speed, otherwise turn on point
         // scale speed with distance to target
         speed = std::min(distanceToTarget, maxDriveSpeed);
         speed = std::max(speed, minDriveSpeed);
         // ... but keep between some min and max values
         // the slowdown is needed so we don't completely overshoot and turn endlessly to reach the point
      }
      else {
         // if dtheta is too large -> only turn, no driving
         dtheta = (dtheta > 0) ? turnSpeed : -turnSpeed;
      }

      // send drive command
      auto dPose = drive(velocity_pub, state, speed, duration, dtheta);

      /**********************
       * KALMAN FILTER HERE *
       **********************/
      
      PoseWithVariances predState, measuredState;

      if (newAmclData) {
         // if new amcl data was received -> set the measuredState and its variances
         measuredState.pose.x = currentAmclData.position.x;
         measuredState.pose.y = currentAmclData.position.y;

         {
            // calculate theta from the measured state's quaternion
            double roll, pitch, yaw;
            auto& ori = currentAmclData.orientation;
            tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
            measuredState.pose.theta = yaw;
         }

         // actually set the variances now that we have data
         measuredState.varx = MeasureVarX;
         measuredState.vary = MeasureVarY;
         measuredState.vartheta = MeasureVarTheta;
  
         newAmclData = false;
      }

      predState.pose.x = state.x + dPose.x;
      predState.pose.y = state.y + dPose.y;
      predState.pose.theta = normalizeAngle(state.theta + dPose.theta);

      predState.varx = ProcessVarX;
      predState.vary = ProcessVarY;
      predState.vartheta = ProcessVarTheta;

      // actual filter call to get new estimated state
      state = kalmanFilter(state, predState, measuredState);

      /*********************
       * KALMAN FILTER END *
       *********************/
      
      // calculate new distance to target
      dx = targetX - state.x;
      dy = targetY - state.y;
      distanceToTarget = std::sqrt(dx * dx + dy * dy);

      std::cout << std::fixed << std::setw(6) << std::setprecision(3);
      std::cout << "\rCurrent State: (" << state.x << ", " << state.y << ", " << state.theta / M_PI * 180. << ") distance to target: " << distanceToTarget << "        " << std::flush;
   }

   std::cout << std::endl;

   // point was reached so set speed to zero
   drive(velocity_pub, state, 0, duration, 0);
}

void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amclPose) {
   // there would be a race condition here but the laser publishes so slowly compared to the refreshrate of
   // the robot that it doesn't matter
   currentAmclData = amclPose->pose.pose;
   newAmclData = true;
}
