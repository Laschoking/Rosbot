#include <ros/ros.h>
#include <ctime>
#include "measure_yaw.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "helpers.h"
#include <cmath>
#include "rosbot_ekf/Configuration.h"
#include <array>
#include "measure_acc.h"

//Input Abfrage
//definiere Punkte = stack v. Positionen
//Problem: Position kann nicht genau angesteuert werden, d.h. System weiß dass Position sich 2cm vom "Soll" unterscheidet,
// würde allerdings nicht da hinkommen, da Bewegungsfehler
using namespace std;
//odom -> X, imu -> lin. acc.
bool need_value[2];
const int odom = 0;
const int imu = 1;
double odom_x;
double imu_x;
double imu_vel;
const double bias = -0.108;
boost::shared_ptr<const sensor_msgs::Imu> old_imu;
int state;
double imu_time;

void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom_msg){
    //cout << "entered_odom"  <<endl;
    if (need_value[odom]){
        if(odom_msg->pose.pose.position.x != 0 && state == 0){
            //cout << "waiting for reset of odom"<<endl;
            return;
        }
            //cout << "entered_odom2"  <<endl;

        odom_x = odom_msg->pose.pose.position.x;
        need_value[odom] = false;
    }
}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    //cout << "entered_imu"  <<endl;

    if(old_imu == nullptr){
        old_imu = imu_msg;
        return;
    }else
    if (need_value[imu]){
        //cout << "entered_odo2m"  <<endl;
        if(imu_msg->header.seq - old_imu->header.seq < 0){
            return;
        }else if(imu_msg->header.seq - old_imu->header.seq > 1){
            cout <<"falsche sequenz reihenfolge " <<  old_imu->header.seq  << "  " << imu_msg->header.seq <<endl;
            cout <<"falsche sequenz reihenfolge " <<  old_imu->header.stamp << "  " << imu_msg->header.stamp <<endl;
            }
        double acc = old_imu->linear_acceleration.x - bias;
        ros::Duration dur = imu_msg->header.stamp - old_imu->header.stamp;
        cout << "duration btw. time stamps: " << dur.toSec();
        //bilde mittelwert, da ja acc_alt ueber Zeit t auf acc_neu anssteigt/ sinkt
        double vel_gain = acc * dur.toSec();
        imu_vel += vel_gain;
        imu_time += dur.toSec();
        imu_x += imu_vel*dur.toSec();
        cout << "vel: " << imu_vel << " acc: " << acc << " gain: " << vel_gain << " total_dist: " << imu_x << endl;
        }
    old_imu = imu_msg;
}




int main(int argc,char **argv) {
    ros::init(argc, argv, "measure_x_accc");
    ros::NodeHandle n("~");
    ros::Subscriber odom_wheel_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",1000,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",1000,imuCallback);
    ros::Publisher move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    bool cont = true;
    ros::Rate loop_rate(10);
    std::ofstream save_values;
    save_values.open("/home/husarion/husarion_ws/src/Beleg/yaw.csv",std::ofstream::app);
    state = 0;
    imu_time = 0;
    double distance;
    double speed;
    while (cont) {
        switch (state){
        case 0:
            if(getBoolInput("Neue X-Messungen?",true)){
                distance = getDoubleInput("Enter distance",1);
                speed = getDoubleInput("Enter speed",0.2);
                old_imu = nullptr;
                imu_vel = 0;
                imu_x = 0;
                odom_x = 0;
                resetImuOdom();
                need_value[odom] = true; // zum pruefen ob x = 0.0
                state = 1;
        }
        else cont = false;
        break;
        case 1:
            if(!need_value[odom]){
                need_value[imu] = true;
                ros::Time t1 = ros::Time::now();
                drive(move_base,distance,speed);
                ros::Time t2 = ros::Time::now();
                need_value[imu] = false;
                cout << "time diff meas: " << (t2-t1).toSec() << " imu: " << imu_time << endl;
                need_value[odom] = true;
                state = 2;
            }
            break;
        case 2:
            if (!need_value[odom]){
               cout << "Ergenisse: odom " << odom_x << " imu: " << imu_x <<endl;
               std::time_t result = std::time(nullptr);
               save_values << distance << "," << odom_x << "," << imu_x << "," << std::asctime(std::localtime(&result)) <<endl;
               if(getBoolInput("zurueck fahren?",true)){
                   drive(move_base, distance, -0.5);
                   state = 0;
               }else cont = false;
            }
        }
      ros::spinOnce();
    }
    save_values.close();
    exit(0);
}