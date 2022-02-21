#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <array>
#include <boost/shared_ptr.hpp>
#include <ros/topic.h>
#include <fstream>
using namespace std;


void laser_distances(boost::shared_ptr<sensor_msgs::LaserScan const> scan){
    float min_range = scan->range_min;
    float max_range = scan->range_max;
    size_t size = scan->ranges.size();
    //ROS_INFO("size of range %f", sizeof(scan->ranges)/sizeof(scan->ranges[0]));
    fstream myfile;
    myfile.open("out_ranges.txt");
    ROS_INFO("amount of ranges %i",size);
    /*for (size_t i = 0 ; i < size; i++){
        float dist = scan->ranges[i];
        ROS_INFO("test distance %f",dist);
        myfile << dist << endl;
    }*/
    myfile.close();
    cout << "file closed";

    //scanSub.unregister();
    /*float ranges [] = scan->ranges;
    for (float &r : ranges){
        cout << r << endl;
    }*/


}
int main(int argc,char **argv){
    cout << "-----------------start saving distances---------";
    ros::init(argc,argv,"subscribe_to_scan");
    ros::NodeHandle nh;
    ros::Subscriber scanSub;
    boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr;
    //sensor_msgs::LaserScan::ConstPtr sharedPtr;
    //sharedPtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",nh);
    //const sensor_msgs::LaserScanConstPtr& scan;
    try {
        sharedPtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",nh,ros::Duration(10));
        laser_distances(sharedPtr);
    }catch(int n) {
        ROS_WARN("Fehler");
    }
    //scanSub = nh.subscribe("/scan", 1, laser_distances);
    ros::spin();
    return 0;

}
