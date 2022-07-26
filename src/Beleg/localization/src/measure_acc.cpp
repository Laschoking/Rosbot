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
bool res_odom  = true;
const int odom = 0;
const int imu = 1;
double odom_x;
double odom_y;
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
        if (!res_odom){
            if (abs(odom_msg->pose.pose.position.x) < 0.05 && abs(odom_msg->pose.pose.position.y) < 0.05 ){
               res_odom = true;
            }
        }
        odom_x = odom_msg->pose.pose.position.x;
        odom_y = odom_msg->pose.pose.position.y;
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
        //cout << "duration btw. time stamps: " << dur.toSec();
        //bilde mittelwert, da ja acc_alt ueber Zeit t auf acc_neu anssteigt/ sinkt
        double vel_gain = acc * dur.toSec();
        imu_vel += vel_gain;
        imu_time += dur.toSec();
        imu_x += imu_vel*dur.toSec();
        //cout << "vel: " << imu_vel << " acc: " << acc << " gain: " << vel_gain << " total_dist: " << imu_x << endl;
        }
    old_imu = imu_msg;
}




int main(int argc,char **argv) {
    ros::init(argc, argv, "measure_x_accc");
    ros::NodeHandle n("~");
    ros::Subscriber odom_wheel_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",1,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",5,imuCallback);
    ros::Publisher move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    bool cont = true;
    ros::Rate loop_rate(10);
    std::ofstream save_values;
    save_values.open("/home/husarion/husarion_ws/src/Beleg/acc.csv",std::ofstream::app);
    state = 0;
    int c = 0;
    imu_time = 0;
    double distance = 1;
    double speed = 0.2;
    distance = getDoubleInput("Enter distance",distance);
    speed = getDoubleInput("Enter speed",speed);
    while (cont) {
        switch (state){
        case 0:
            if(getBoolInput("Neue X-Messungen?",true)){
                old_imu = nullptr;
                imu_vel = 0;
                imu_x = 0;
                odom_x = 0;
                resetImuOdom();
                res_odom = false;
                need_value[odom] = true; // zum pruefen ob x = 0.0
                state = 1;
        }
        else cont = false;
        break;
        case 1:
            if(!need_value[odom] && res_odom){
                need_value[imu] = true;
                ros::Time t1 = ros::Time::now();
                drive(move_base,distance,speed);
                ros::Time t2 = ros::Time::now();
                need_value[imu] = false;
                ros::Duration(1).sleep();
                //cout << "time diff meas: " << (t2-t1).toSec() << " imu: " << imu_time << endl;
                need_value[odom] = true;
                state = 2;
            }
            break;
        case 2:
            if (!need_value[odom]){
               c++;
               cout << "Ergnis nr. "<< c << " gespeichert: odom_x " << odom_x << "y:" << odom_y<< " imu: " << imu_x <<endl;
               std::time_t result = std::time(nullptr);
               double x_meas = getDoubleInput("x-Messung",0);
               double y_meas = getDoubleInput("y-Messung",0);
               save_values << distance << "," << speed << "," << x_meas << "," << y_meas <<  "," << odom_x << "," << odom_y << "," << imu_x << "," << std::asctime(std::localtime(&result)) <<"\n";
               if(getBoolInput("zurueck fahren",true)){
                   drive(move_base, distance - 0.02, -0.5);
                   state = 0;
               }else cont = false;
            }
        }
      ros::spinOnce();
      loop_rate.sleep();
    }
    save_values << flush;
    save_values.close();
    exit(0);
}