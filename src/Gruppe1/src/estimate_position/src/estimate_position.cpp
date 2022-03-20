#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <array>
#include <iostream>
#include<typeinfo>
#include <math.h>
#include <random>
#include <cmath>
using namespace std;
array<double,3> last_position = {0,0,0};
//kommando zeile option: d= distanz zum start, s neuer start, pos = current position
//slam_gmapping -> erzeuge Karte nutzt odometry & laser scan -> hier parameter anpassen???
array<double,3> pos_to_array(const nav_msgs::Odometry::ConstPtr& msg){
    const geometry_msgs::Point pos = msg->pose.pose.position;

    /* for use in gazebo, to create some normal distributed noise
     * std::random_device rd{};
     * std::mt19937 gen{rd()};
     * normal_distribution<> d{0, 0.01};
     * array<double,3> current_position = {pos.x + d(gen),pos.y + d(gen),pos.z};
     */
    array<double,3> current_position = {pos.x ,pos.y ,pos.z};

    return current_position;
}
void posCallback(const nav_msgs::Odometry::ConstPtr& msg){
    array< double,3> current_position = pos_to_array(msg);
    ROS_INFO("beide positionen gleich? %d \n", current_position == last_position);
    ROS_INFO("current_position x: %lf,y %lf,z %lf:",current_position[0],current_position[1],current_position[2]);
    if (current_position != last_position){
        float dist = sqrt(pow(current_position[0],2) + pow(current_position[1],2));
        ROS_INFO("distanz zum Startpunkt: %f m", dist);
        last_position = current_position;
    }
}

int main(int argc,char **argv){
    ros::init(argc, argv, "odom_sub_node");
    ros::NodeHandle nh;
    ros::Rate rate(1);

    ros::Subscriber sub = nh.subscribe("/odom",1000, posCallback);
    rate.sleep();
    ros::spin();
    return 0;
}
