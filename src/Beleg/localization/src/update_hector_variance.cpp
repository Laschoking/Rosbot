#include <array>
#include <fstream>
#include <cmath>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <boost/shared_ptr.hpp>

#include "helpers.h"

using namespace std;

geometry_msgs::Pose lasthectorPose;
int seq = 0;

void hectorCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> hectorPose) {

}

int main(int argc,char **argv){
    ros::init(argc, argv, "subscribe_to_scan");
    ros::NodeHandle n("~");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ros::Subscriber hector_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/poseupdate", 1, hectorCallback);



    // wait again for hector to definitely have published its last data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    exit(0);
}
