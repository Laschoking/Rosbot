#include <array>
#include <fstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <boost/shared_ptr.hpp>
//#include <Eigen3/Eigen/Dense>
//#include <Eigen3/Eigen/LU>

using namespace std;
//using namespace Eigen;
ros::Publisher hector_pub;


void hectorCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> poseupdate) {
    geometry_msgs::PoseWithCovariance h_pos=  poseupdate->pose;
    //cout << "original covariance" << poseupdate->pose.covariance[0] << endl;
    int i = 0;
    int count_zero_cov = 0;
/*
    boost::array<double,6> var = {h_pos.covariance[0],h_pos.covariance[7],h_pos.covariance[14],h_pos.covariance[21],h_pos.covariance[28],h_pos.covariance[35]};
    double* min_val =  min_element(var.begin(), var.end());
    float norm;
    if (*min_val < 0){
        norm = abs(*min_val);
    }else{
        norm = 0;
    }
    //cout << "norm factor " << norm << endl;
    for (int j= 0; j < 6; j++){
        if (var[j] != 0){
            var[j] = 0.1/(var[j] + norm + pow(10,-9));
        }else{
        var[j] = 0;
        count_zero_cov++;
        }
    }


    boost::array<double,36> cov;
    while (i < 36){
        if (i % 7 == 0){
            cov[i] = var[i/7];
        }else{
        cov[i] = 0;
        }
        i++;
    }

    geometry_msgs::PoseWithCovarianceStamped new_pos;
    new_pos.header = poseupdate->header;
    new_pos.pose.pose = poseupdate->pose.pose;
    new_pos.pose.covariance = cov;
*/
    hector_pub.publish(poseupdate);


}
int main(int argc,char **argv){
    ros::init(argc, argv, "subscribe_to_scan");
    ros::NodeHandle n("~");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    hector_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/scanmatch_new",1000);
    ros::Subscriber hector_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/scanmatch_odom", 1000, hectorCallback);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep(); // Don't forget this! *
    }

    // wait again for hector to definitely have published its last data
    /*std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ros::spin();*/
    exit(0);
}
