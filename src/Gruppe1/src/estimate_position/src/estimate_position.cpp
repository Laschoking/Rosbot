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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "driver.h"
#include "helpers.h"

using namespace std;

using position = std::tuple<double, double>;

int main(int argc,char **argv) {
    // initialize the ros systems
    ros::init(argc, argv, "drive_to_point");
    ros::NodeHandle n("~");
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber amcl_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, amclCallback);

    // wait a little so everything is ready and done printing to console
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // set initial state
    geometry_msgs::Pose2D state;
    state.x = 0;
    state.y = 0;
    state.theta = 0;

    if (getBoolInput("Use predefined Path", true)) {
        // put some points to drive to
        std::queue<position> targets;
        targets.push(position{0, 1});
        targets.push(position{1, 0});
        targets.push(position{1.5, 0.5});
        targets.push(position{-0.5, 1.5});
        targets.push(position{0.25, -0.5});
        targets.push(position{0, 0});
            
        while (!targets.empty()) {
            auto nextTarget = targets.front();
            targets.pop();
            driveToPoint(velocity_pub, std::get<0>(nextTarget), std::get<1>(nextTarget), state);
            std::cout << "Press Enter to continue..." << std::flush;
            std::cin.get();
        }
    }
    else {
        // wait for user to input coordinates to drive to
        do {
            double x = getDoubleInput("Next target x value");
            double y = getDoubleInput("Next target y value");

            driveToPoint(velocity_pub, x, y, state);

        } while (getBoolInput("Continue", true));
    }

    exit(0);
}
