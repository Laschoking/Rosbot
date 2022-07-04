#include <ros/ros.h>
#include "measure_yaw.h"
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include "helpers.h"
#include <cmath>
#include "rosbot_ekf/Configuration.h"
#include <tf/transform_datatypes.h>
#include <array>

//void manage_states(double yaw, double source);
//void rotate_vehicle(double angle, double distance);
bool need_value[2] =  {false,false}; // 0 = nein, 1 = ja, 2 = schon gemacht 3: neue Werte Speichern
int imu = 1;
int odom = 0;
double values[2][2] = {{0,0},{0,0}};

rosbot_ekf::Configuration configuration_msg;
double angle = 180;
ros::Publisher move_base;
enum States {setup,get_old_values,get_new_values};
States state = setup;

using namespace std;

void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom_msg){
    if (need_value[odom]){
        geometry_msgs::Quaternion ori = odom_msg->pose.pose.orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        manage_states(yaw,odom); //TODO kann yaw auch negative Werte annehmen??
        }
}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    if (need_value[imu]){
        geometry_msgs::Quaternion ori = imu_msg->orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        manage_states(yaw,imu);
        }
}


void manage_states(double yaw, bool source){
    switch (state){
        case setup:
            configuration_msg.request.command = "RODOM";
            configuration_msg.request.data = "";
            ros::service::call("config", configuration_msg);
            configuration_msg.request.command = "RIMU";
            ros::service::call("config", configuration_msg);
            cout << "Reset Raw Odometry and IMU" << endl;
            angle = getDoubleInput("Winkelangabe in Grad?",angle);
            need_value[imu] = true;
            need_value[odom] = true;
            state = get_old_values;
            break;
        case get_old_values:
            values[0][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                rotate(move_base,degree_to_yaw(angle));
                drive(move_base,0.5,0.2);
                state = get_new_values;
                need_value[imu] = true;
                need_value[odom] = true;
                }
            break;
        case get_new_values:
            values[1][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                cout << "Differenz von Rotation & Bewegung: " << endl;
                double diff_yaw[2] = {int (yaw_to_degree(values[1][odom] - values[0][odom])) % 360, int (yaw_to_degree(values[1][imu] - values[0][imu]))%360};
                cout << "Odom_yaw in Grad: " << diff_yaw[0] << endl;
                cout << "Imu_yaw in Grad: " << diff_yaw[1] << endl;
                //TODO write results to csv table
                cout << "Werte in CSV-Datei gespeichert" <<endl;
                if(getBoolInput("Zurueckfahren + neue Messung?",true)){
                    rotate(move_base,M_PI);
                    drive(move_base,0.5,0.2);
                    //rotate() back
                    state = setup;
                    manage_states(0,0);
                }else {
                    exit(0);
                 }
             }
            break;
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
        state = setup;
        manage_states(0,0);
    }
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep(); // Don't forget this! *
    }
    exit(0);
}