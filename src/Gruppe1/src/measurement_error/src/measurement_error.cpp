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

geometry_msgs::Pose lastAmclPose;
int seq = 0;

void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amclPose) {
   lastAmclPose = amclPose->pose.pose;
   seq = amclPose->header.seq;
}

int main(int argc,char **argv){
    ros::init(argc, argv, "subscribe_to_scan");
    ros::NodeHandle n("~");

    // wait for amcl to be ready before doing anything
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber amcl_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, amclCallback);

    drive(velocity_pub, 1, 0.25);

    // wait again for amcl to definitely have published its last data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // output last amcl data
    double yaw;
    {
        // calculate theta from the measured state's quaternion
        double roll, pitch;
        auto& ori = lastAmclPose.orientation;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        yaw = yaw / M_PI * 180;
    }

    std::cout << "Last AMCL Data: " << seq << ", " << lastAmclPose.position.x << ", " << lastAmclPose.position.y << ", " << yaw << std::endl;
    
    exit(0);
}
