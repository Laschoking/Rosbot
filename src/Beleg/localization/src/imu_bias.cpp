#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <sensor_msgs/Imu.h>
#include <cmath>

double lin_bias = 0;
double ang_bias = 0;
int count;
bool measure;
void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    if(measure){
        lin_bias += imu_msg->linear_acceleration.x;
        ang_bias += imu_msg->angular_velocity.z;
        //std::cout << lin_bias << " " <<ang_bias << std::endl;
        count ++;
    }
}


int main(int argc,char **argv) {
    ros::init(argc, argv, "measure_x_accc");
    ros::NodeHandle n("~");
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",1000,imuCallback);
    ros::Rate loop_rate(10);
    ros::Time t = ros::Time::now() + ros::Duration(120,0);
    measure = true;
    while (ros::Time::now() < t){
        loop_rate.sleep();
        ros::spinOnce();
    }
    measure = false;

    std::cout <<count <<  "  imu_bias lin acc: " << lin_bias/count << std::endl;
    std::cout << "imu_bias ang. vel: " << ang_bias/count << std::endl;

    exit(0);
}
