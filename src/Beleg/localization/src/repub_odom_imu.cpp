#include "repub_odom_imu.h"

void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg,ros::Publisher* odom_repub){
    nav_msgs::Odometry tmp = *odom_msg;
    tmp.pose.covariance[0] = 0.5;//meas_const.odom_x_var;
    tmp.pose.covariance[35] = 0.5;//meas_const.odom_yaw_var;
    odom_repub->publish(tmp);
}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg, ros::Publisher* imu_repub){
    sensor_msgs::Imu tmp = *imu_msg;
    tmp.orientation_covariance[8] = 0.001;
    tmp.angular_velocity_covariance[8] = 0.001;
    imu_repub->publish(tmp);
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