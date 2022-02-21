#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>


void laser_distances(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("position=: [%f]", scan->scan_time);

}
int main(int argc,char **argv){
    ros::init(argc,argv,"subscribe_to_scan");
    ros::NodeHandle nh;
    ros::Subscriber scanSub;
    scanSub = nh.subscribe("/scan",1000, laser_distances);
    ros::spin();
}
