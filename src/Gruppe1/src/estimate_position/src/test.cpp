#include <array>
#include <iostream>
#include <typeinfo>
#include <math.h>
#include <random>
#include <cmath>

#include "driver.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

int main(int argc,char **argv) {
    ros::init(argc, argv, "drive_to_point");
    ros::NodeHandle n("~");

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Pose2D state;
    state.x = 0;
    state.y = 0;
    state.phi = 0;

    ros::Publisher velocity_pub;

    driveToPoint(velocity_pub, 1, 1.5, state);

    std::cout << "state x: " << state.x << " y: " << state.y << " phi: " << state.phi << std::endl;
}

/*
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
int main()
{
    std::random_device rd{};
    std::mt19937 gen{rd()};

    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d{5,2};

    std::map<int, int> hist{};
    for(int n=0; n<10000; ++n) {
        ++hist[std::round(d(gen))];
    }
    for(auto p : hist) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    }
}*/