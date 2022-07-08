#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include "helpers.h"
#include "rosbot_ekf/Configuration.h"
#include <thread>
using namespace std;

void resetImuOdom(){
        rosbot_ekf::Configuration configuration_msg;
        configuration_msg.request.command = "RODOM";
        configuration_msg.request.data = "";
        ros::service::waitForService("config");
        if( ros::service::call("config", configuration_msg)){
            cout << "reset odom" << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        /*configuration_msg.request.command = "RIMU";
        ros::service::waitForService("config");
        if (ros::service::call("config", configuration_msg)){
            cout << "reset IMU" << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }*/
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

      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(REFRESHRATE) >= duration) {
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

   int counter = 0;

   double duration = abs(yaw) * 3 / M_PI ;
   yaw += abs(yaw) > M_PI ? (yaw > M_PI ? -2*M_PI : 2*M_PI) : 0; // wenn winkel > 180 bzw kleiner als -180 ist, rotiere in andere Richtung

   auto current_yaw = yaw / duration;

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

      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(REFRESHRATE) >= duration) {
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


