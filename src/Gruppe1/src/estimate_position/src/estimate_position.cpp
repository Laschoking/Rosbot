#include <array>
#include <iostream>
#include <typeinfo>
#include <math.h>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include "driver.h"

using namespace std;

using position = std::tuple<double, double>;

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

    if (current_position != last_position) {
    	ROS_INFO("current_position x: %lf,y %lf,z %lf:",current_position[0],current_position[1],current_position[2]);
    }

    if (current_position != last_position) {
        float dist = sqrt(pow(current_position[0],2) + pow(current_position[1],2));
        ROS_INFO("distanz zum Startpunkt: %f m", dist);
        last_position = current_position;
    }
}

int main(int argc,char **argv){

    ros::init(argc, argv, "drive_to_point");
    ros::NodeHandle n("~");

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    std::queue<position> targets;
    targets.push(position{0, 1});
    targets.push(position{1, 0});
    targets.push(position{1.5, 0.5});
    targets.push(position{-0.5, 1.5});
    targets.push(position{0.25, -0.5});
    targets.push(position{0, 0});

    geometry_msgs::Pose2D state;
    state.x = 0;
    state.y = 0;
    state.theta = 0;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (!targets.empty()) {
        auto nextTarget = targets.front();
        targets.pop();
        driveToPoint(velocity_pub, std::get<0>(nextTarget), std::get<1>(nextTarget), state);
        std::cout << "state x: " << state.x << " y: " << state.y << " theta: " << state.theta << std::endl;
    }

    exit(0);
}
