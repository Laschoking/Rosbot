#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include "helpers.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "rosbot_ekf/Configuration.h"
#include <thread>
#include <array>
using namespace std;
/*
 * reset odom/wheel -> rosservice call /conifg RODOM ''
 * reset odom (ekf) -> rosserrvice call /set_pose  ! orientation muss passen
 * reset amcl -> rostopic pub /initial_pose #orientierung beachten
 * */
void resetImuOdom(){
        rosbot_ekf::Configuration configuration_msg;
        configuration_msg.request.command = "RODOM";
        configuration_msg.request.data = "";
        ros::service::waitForService("config");
        if( ros::service::call("config", configuration_msg)){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        configuration_msg.request.command = "RIMU";
        ros::service::waitForService("config");
        if (ros::service::call("config", configuration_msg)){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
}

void resetEKF(ros::Publisher* reset_ekf){
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y= 0;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;
    msg.pose.pose.orientation.w = 1;
    boost::array<double,36> cov = {0.0518246686120483, -0.0006410850085873191, 0.0, 0.0, 0.0, 0.0, -0.0006410850085873185, 0.06223791701757981, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05051411214006334};
    msg.pose.covariance = cov;
    reset_ekf->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void resetAMCL(ros::Publisher* reset_amcl){
    geometry_msgs:: PoseWithCovarianceStamped msg;
    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y= 0;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation.x  =0;
    msg.pose.pose.orientation.y  =0;
    msg.pose.pose.orientation.z  =0;
    msg.pose.pose.orientation.w = 1;
    boost::array<double,36> cov = {0.0518246686120483, -0.0006410850085873191, 0.0, 0.0, 0.0, 0.0, -0.0006410850085873185, 0.06223791701757981, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05051411214006334};
    msg.pose.covariance = cov;
    reset_amcl->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

double yaw_to_degree(double yaw){
 
    return (yaw*180)/M_PI;
    }
double degree_to_yaw(double degree){
    return (degree/180)*M_PI;
    }

// drive the given distance with given constant speed, then stop
void drive(ros::Publisher& velocity_pub, const double distance, const double speed) {
   const int REFRESHRATE = 20;
   ros::Rate loop_rate(REFRESHRATE);

   int counter = 0;

   double duration = distance / abs(speed);


   while (ros::ok()) {
      ros::spinOnce();

      ++counter;
      geometry_msgs::Twist velocity;

      velocity.linear.x = speed;
      velocity.linear.y = 0;
      velocity.linear.z = 0;

      velocity.angular.x = 0;
      velocity.angular.y = 0;
      velocity.angular.z = 0;
      //first and last message with half speed
      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(REFRESHRATE) + 1/REFRESHRATE >= duration) {
         break;
      }

      loop_rate.sleep();
   }

   geometry_msgs::Twist velocity;

   velocity.linear.x = 0;
   velocity.linear.y = 0;
   velocity.linear.z = 0;

   velocity.angular.x = 0;
   velocity.angular.y = 0;
   velocity.angular.z = 0;

   // stop motors
   velocity_pub.publish(velocity);
}


void rotate(ros::Publisher& velocity_pub, double yaw) {
   const int REFRESHRATE = 20;
   ros::Rate loop_rate(REFRESHRATE);

   int counter = -1;
    //only positive yaw values ingoing -> rotation clockwise if yaw too big
   if (yaw > 2*M_PI){
       yaw = fmod(yaw,2*M_PI);
   }else {
       if (yaw > M_PI) {
           yaw = yaw - 2*M_PI;
       }else if(yaw < 0){
           if (yaw < 2*-M_PI){
               cout << "negative yaw received in rotation: " << yaw << "\n";
               yaw = fmod(yaw,2*M_PI);
           }else if (yaw < -M_PI){
               cout << "negative yaw received in rotation: " << yaw << "\n";
               yaw = 2*M_PI- yaw;
           }
       }
   }
   auto current_yaw = yaw >= 0 ? 0.5 : -0.5;
   double duration = yaw /current_yaw;

   //cout << "dauer der Drehung: " << duration << " drehung insgesamt um: " << yaw << "\n";
   while (ros::ok()) {
      ros::spinOnce();

      ++counter;

      geometry_msgs::Twist velocity;

      velocity.linear.x = 0;
      velocity.linear.y = 0;
      velocity.linear.z = 0;

      velocity.angular.x = 0;
      velocity.angular.y = 0;
      velocity.angular.z = current_yaw;
      if (counter == 0 || (counter + 1)  / static_cast<double>(REFRESHRATE) + 1/REFRESHRATE >= duration){
          velocity.angular.z = current_yaw/2;
      }
      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(REFRESHRATE) + 1/REFRESHRATE >= duration) {
         break;
      }
      loop_rate.sleep();
   }

   geometry_msgs::Twist velocity;

   velocity.linear.x = 0;
   velocity.linear.y = 0;
   velocity.linear.z = 0;

   velocity.angular.x = 0;
   velocity.angular.y = 0;
   velocity.angular.z = 0;

   // stop motors
   velocity_pub.publish(velocity);
}



double getDoubleInput(const std::string& question, const double def) {
   bool retry = true;
   double result = 0;

   std::cout << question << "? (double value, default: " << def << "): ";

   while (retry) {
      retry = false;
      
      std::string ans;
      std::getline(std::cin, ans);

      if (ans.empty()) {
         result = def;
         std::cout << "Using default value: " << def << std::endl;
         break;
      }

      std::stringstream ss(ans);
      retry = (ss >> result).fail() || std::cin.fail() || !ss.eof() || ans.empty();

      if (retry) {
         std::cout << "Please enter a valid number!" << std::endl;
         retry = true;
      }
   }

   return result;
}

bool getBoolInput(const std::string& question, const bool def) {
   bool retry = true;
   bool result = false;

   std::cout << question << "? (y/n, - default: " << (def ? "y" : "n") << "): ";

   while (retry) {
      retry = true;

      std::string ans;
      std::getline(std::cin, ans);

      if (ans.empty()) {
         result = def;
         std::cout << "Using default value: " << (def ? "y" : "n") << std::endl;
         retry = false;
         //break;
      }
      else if (ans == "y" || ans == "Y" || ans == "yes" || ans == "1") {
         result = true;
         retry = false;
      }
      else if (ans == "n" || ans == "N" || ans == "no" || ans == "0") {
         result = false;
         retry = false;
      }

      std::cin.clear();
   }

   return result;
}


