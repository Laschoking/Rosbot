#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "helpers.h"
#include <cmath>
#include "rosbot_ekf/Configuration.h"
#include <boost/shared_ptr.hpp>
#include <tf/transform_datatypes.h>
#include <array>

bool need_value[2] =  {false,false}; // 0 = nein, 1 = ja, 2 = schon gemacht 3: neue Werte Speichern
bool imu = 1;
bool odom = 1;
double values[2][2] = {{0,0},{0,0}};

rosbot_ekf::Configuration configuration_msg;
double angle = 180;
ros::Publisher move_base;
enum states {"setup","get_old_values","get_new_values"};
states state;

using namespace std;

void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom_msg){
    if (need_value[odom]){
        geometry_msgs::Quaternion ori = odom_msg->pose.pose.orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        double odom_yaw = yaw / 180*M_PI ;
        manage_state(odom_yaw,odom); //TODO kann yaw auch negative Werte annehmen??
        }

}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    if (need_value[imu]){
        geometry_msgs::Quaternion ori = imu_msg->pose.pose.orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        manage_state(yaw / 180*M_PI,imu);
        }
}
void manage_states(double yaw, bool source){
    switch (state){
        case "setup":
            configuration_msg.request.command = "RODOM";
            configuration_msg.request.data = "";
            ros::service::call("config", configuration_msg);
            configuration_msg.request.command = "RIMU";
            ros::service::call("config", configuration_msg);
            cout << "Reset Raw Odometry and IMU" << endl;
            angle = getDoubleInput("Winkelangabe in Grad?",angle);
            need_imu = true;
            need_odom = true;
            state = "get_old_value";
            break;
        case "get_old_values":
            values[0][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                rotate_vehicle(angle,0.5);
                state="get_new_values";
                }
            break;
        case "get_new_values":
            values[1][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                cout << "Differenz von Rotation & Bewegung: " << endl;
                double[2] diff_yaw = {values[1][0] - values[0][0], values[1][i] - values[0][1]};
                cout << "Odom_yaw in Grad: " << diff_yaw[0] << endl;
                cout << "Imu_yaw in Grad: " << diff_yaw[1] << endl;
                //TODO write results to csv table
                cout << "Werte in CSV-Datei gespeichert" <<endl;
                if(getBoolInput("Zurueckfahren?",true){
                    rotate(180,0.5);
                    //rotate() back
                    state = "setup";
                    manage_states(0,0);
                }
            }
            break;
    }


void rotate_vehicle(double angle, double distance){
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = angle*M_PI/180;
        move_base.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        msg.angular.z = 0;
        msg.linear.x = distance;
        move_base.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

int main(int argc,char **argv) {
    // initialize the ros systems
    ros::init(argc, argv, "turn_and_move");
    ros::NodeHandle n("~");
    ros::Subscriber odom_wheel_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",10,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",10,imuCallback);
    move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    if (getBoolInput("Starte Rotationsmessung?",true)){
        state = "setup";
        rotate(0,0);
    }
    ros::Rate loop_rate(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep(); // Don't forget this! *
    }
    exit(0);
}