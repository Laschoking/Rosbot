#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include "helpers.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "rosbot_ekf/Configuration.h"
#include <array>
#include <std_srvs/Empty.h>



//#include <sqlite3.h>
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
        ros::spinOnce();
}
int requestAmclUpdate(){
    std_srvs::Empty msg;
    ros::service::waitForService("request_nomotion_update");
    if(ros::service::call("request_nomotion_update",msg)){
        ros::Duration(0.1).sleep();
        return 1;
    }else return 0;
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
    ros::spinOnce();
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
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

double yaw_to_degree(double yaw){

    return (yaw*180)/M_PI;
    }
double degree_to_yaw(double degree){
    return (degree/180)*M_PI;
    }
double getYawDiff(geometry_msgs::Quaternion* or_1, geometry_msgs::Quaternion* or_2){
    double roll_1,pitch_1,yaw_1;
    tf::Matrix3x3(tf::Quaternion{or_1->x, or_1->y, or_1->z, or_1->w}).getRPY(roll_1, pitch_1, yaw_1);
    double roll_2,pitch_2,yaw_2;
    tf::Matrix3x3(tf::Quaternion{or_2->x, or_2->y, or_2->z, or_2->w}).getRPY(roll_2, pitch_2, yaw_2);
    return abs(yaw_1 - yaw_2);
}

double getYawOffset(geometry_msgs::Pose* curr_pose, geometry_msgs::Point* goal_point){
    geometry_msgs::Quaternion ori = curr_pose->orientation;
    double s_roll,s_pitch,s_yaw;
    tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(s_roll, s_pitch, s_yaw);
    if (s_yaw != s_yaw){
        std::cout << "start yaw was nan, set to 0\n";
        s_yaw = 0;
    }
    double yaw;
    yaw = atan2(goal_point->y- curr_pose->position.y ,goal_point->x- curr_pose->position.x);
    if (yaw != yaw ){
        std::cout << "yaw was nan, set to 0 \n" << yaw;
        std::cout << goal_point->y << curr_pose->position.y << goal_point->x  << curr_pose->position.x << std::endl;
        yaw = 0;}
    //std::cout << "calculate yaw offset" << s_yaw << " ziel richtung: " << yaw << " diff: " << yaw - s_yaw << std::endl;
    return yaw - s_yaw; //rotiere um yaw = abweichung v. X-Koordinate + offset zur x-koordinate
}

double getXOffset(geometry_msgs::Pose* curr_pose, geometry_msgs::Point* goal_point){
    double d_x, d_y;
    d_x =  goal_point->x - curr_pose->position.x;
    d_y =  goal_point->y - curr_pose->position.y;
    return sqrt(pow(d_x,2)+pow(d_y,2));
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
