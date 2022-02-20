#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#scope std


void laser_distances(ros::Nodehandle nh){
    std::cout << nh;


}
int main(){
    ros::NodeHandle nh;
    ros::Subscriber scanSub;
    scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&AutoExp::processLaserScan,this);
    laser_distances(nh);
}
