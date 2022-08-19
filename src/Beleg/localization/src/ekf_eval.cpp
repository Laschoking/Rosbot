#include <ros/ros.h>
#include "sqlite_lib.h"
#include <cmath>
#include <sqlite3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>


sqlite3* db;
bool new_imu;
bool new_odom;
bool new_ekf;
double imu_start_offset = 0;
bool start = true;
geometry_msgs::Pose odom_pose;
geometry_msgs::Pose ekf_pose;
geometry_msgs::Quaternion imu_quat;

void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg){
    if (!new_odom){
        odom_pose = odom_msg->pose.pose;
        new_odom = true;
    }
}
void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    if (start){
        geometry_msgs::Quaternion ori = imu_msg->orientation;
        double roll,pitch;
        start = false;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, imu_start_offset);
        std::cout << "start Imu offset: " << imu_start_offset <<std::endl;
    }
    if (!new_imu){
        imu_quat = imu_msg->orientation;
        new_imu = true;
    }
}

void ekfCallback(boost::shared_ptr< const nav_msgs::Odometry> ekf_msg){
    if (!new_ekf){
        ekf_pose = ekf_msg->pose.pose;
        new_ekf = true;
    }

}
int main(int argc,char **argv){
    ros::init(argc, argv, "drive_to_points");
    ros::NodeHandle n("~");
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom_new",10,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu_new",10,imuCallback);
    ros::Subscriber ekf_sub = n.subscribe<nav_msgs::Odometry>("/ekf",10,ekfCallback);
    std::string db_file = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db";
    if(sqlite3_open(db_file.c_str(),&db) != SQLITE_OK){
          printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
    }
   std::string sql = "SELECT iteration FROM ekf_eval ORDER BY iteration DESC LIMIT 1;";
   double tmp = getSQLiteOut(db,&sql);
   int iteration;
   if(tmp != tmp){
         iteration = 0;
   }else{
       iteration = (int)tmp;
       iteration++;

   }
    ros::Rate sleep_rate(10);
    int count = 0;
    while(ros::ok){
        ros::spinOnce();
        if(new_odom&& new_ekf && new_imu){
            geometry_msgs::Quaternion ori_odom = odom_pose.orientation;
            double roll_odom,pitch_odom,yaw_odom;
            tf::Matrix3x3(tf::Quaternion{ori_odom.x, ori_odom.y, ori_odom.z, ori_odom.w}).getRPY(roll_odom, pitch_odom, yaw_odom);
            geometry_msgs::Quaternion ori_ekf = ekf_pose.orientation;
            double roll_ekf,pitch_ekf,yaw_ekf;
            tf::Matrix3x3(tf::Quaternion{ori_ekf.x, ori_ekf.y, ori_ekf.z, ori_ekf.w}).getRPY(roll_ekf, pitch_ekf, yaw_ekf);
            geometry_msgs::Quaternion ori_imu = imu_quat;
            double roll_imu,pitch_imu,yaw_imu;
            tf::Matrix3x3(tf::Quaternion{ori_imu.x, ori_imu.y, ori_imu.z, ori_imu.w}).getRPY(roll_imu, pitch_imu, yaw_imu);
            yaw_imu -= imu_start_offset;

            if (yaw_imu < -2*M_PI) yaw_imu += 2*M_PI;
            if (yaw_imu > 2*M_PI) yaw_imu -= 2*M_PI;
            if (yaw_imu > M_PI) yaw_imu = yaw_imu -2*M_PI;
            if (yaw_imu < -M_PI) yaw_imu += 2*M_PI;
            std::string sql = "INSERT INTO ekf_eval VALUES(" + std::to_string(iteration) + "," + std::to_string(count) + "," + std::to_string(ekf_pose.position.x)
            + "," +  std::to_string(ekf_pose.position.y) + "," + std::to_string(odom_pose.position.x) + "," + std::to_string(odom_pose.position.y) + "," +
            std::to_string(yaw_odom) + "," + std::to_string(yaw_ekf) + "," + std::to_string(yaw_imu) +");";
            insertSQLite(db,&sql);
            new_odom = new_ekf = new_imu= false;
            count++;
        }
    sleep_rate.sleep();
    }

    sqlite3_close(db);
}