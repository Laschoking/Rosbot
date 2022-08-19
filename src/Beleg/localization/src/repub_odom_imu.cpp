#include "repub_odom_imu.h"
#include "helpers.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
geometry_msgs::Pose old_odom;
geometry_msgs::Quaternion old_imu_or;
//const double x_offset = 0.005;
//const double yaw_offset = 0.005;


void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg,ros::Publisher* odom_repub){
 //   geometry_msgs::Pose cur_pose = odom_msg->pose.pose;
  //  if (getXOffset(&cur_pose,&old_odom.position)> x_offset || getYawDiff(&cur_pose.orientation,&old_odom.orientation) > yaw_offset){
        nav_msgs::Odometry tmp = *odom_msg;
        tmp.pose.covariance[0] = 6.59418396241594e-06; //meas_const.odom_x_var;
        tmp.pose.covariance[7] = 0.00011676237024532;
        tmp.pose.covariance[35] = 0.00101425160795909;//meas_const.odom_yaw_var;
        odom_repub->publish(tmp);
   // } was used to only update msg when x-diff was bigger than 5mm
    //old_odom = odom_msg->pose.pose;
}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg, ros::Publisher* imu_repub){
   // geometry_msgs::Quaternion cur_or = imu_msg->orientation;
    //if(getYawDiff(&cur_or,&old_imu_or)> yaw_offset){
        sensor_msgs::Imu tmp = *imu_msg;
        tmp.orientation_covariance[8] =  0.00117190176298063; //imu_yaw_var
        tmp.angular_velocity_covariance[8] = 0.0154674185084969; //imu_int_var(für ang_vel)
        imu_repub->publish(tmp);
    //}
    //old_imu_or = imu_msg->orientation;
}

int main(int argc,char **argv){
    ros::init(argc, argv, "repub_odom_imu");
    ros::NodeHandle n("~");
    ros::Publisher odom_new = n.advertise<nav_msgs::Odometry>("/odom_new",1000);
    ros::Publisher imu_new = n.advertise<sensor_msgs::Imu>("/imu_new",1000);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",100,boost::bind(odomCallback,_1,&odom_new));
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",100, boost::bind(imuCallback,_1,&imu_new));
    ros::Rate sleep_rate(10);
    while(ros::ok){
        ros::spinOnce();
        sleep_rate.sleep();
    }
    exit(0);
}