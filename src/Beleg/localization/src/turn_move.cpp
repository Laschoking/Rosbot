#include "ros/ros.h"
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

bool save_odom;
bool save_imu;
double odom_base_yaw;
double imu_base_yaw;

using namespace std;
void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom){
    if (save_odom){
        geometry_msgs::Quaternion ori = odom->pose.pose.orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        odom_base_yaw = yaw / 180*M_PI;
        save_odom = false;
    }
}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu){
    if (save_imu){
        geometry_msgs::Quaternion ori = imu->orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        imu_base_yaw = yaw / 180*M_PI;
        save_imu = false;
    }
}

int main(int argc,char **argv) {
    // initialize the ros systems
    ros::init(argc, argv, "turn_and_move");
    ros::NodeHandle n("~");
    ros::Subscriber odom_wheel_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",10,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",10,imuCallback);
    ros::Publisher move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    rosbot_ekf::Configuration configuration_msg;
    ros::Rate loop_rate(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (getBoolInput("Starte Rotationsmessung?",true)){
        double angle = getDoubleInput("Winkelangabe in Grad",180);
        configuration_msg.request.command = "RODOM";
        configuration_msg.request.data = "";
        ros::service::call("config", configuration_msg);
        configuration_msg.request.command = "RIMU";
        ros::service::call("config", configuration_msg);
        cout << "Reset Raw Odometry and IMU" << endl;
        save_imu = save_odom = true;
        if (!save_imu && !save_odom){
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = angle*M_PI/180;
                move_base.publish(msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                msg.angular.z = 0;
                msg.linear.x = 0.1;
                move_base.publish(msg);
        }

    }
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep(); // Don't forget this! *
    }


    exit(0);
}