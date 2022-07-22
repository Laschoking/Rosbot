#include <ros/ros.h>
#include <ctime>
#include "drive_to_points.h"
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <sstream>
#include <chrono>
#include <thread>
#include "helpers.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <array>
bool need_value[3];
bool res_src[3];
constexpr int odom = 0;
constexpr int ekf = 1;
constexpr int amcl = 2;
const double speed = 0.2;
const double theta_yaw = 0.5; //radians
const double theta_x = 0.10;  //meters
geometry_msgs::Point odom_pose;
geometry_msgs::Point ekf_pose;
geometry_msgs::Pose amcl_pose;

using namespace std;


void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg){
    if(need_value[odom]){
        odom_pose = odom_msg->pose.pose.position;
        need_value[odom] = false;
    }
    if(!res_src[odom]){
        if (abs(odom_msg->pose.pose.position.x) < 0.05 && abs(odom_msg->pose.pose.position.y) < 0.05 ){
        res_src[odom] = true;
        }
    }
}
void ekfCallback(boost::shared_ptr< const nav_msgs::Odometry> ekf_msg){
    if(need_value[ekf]){
        ekf_pose = ekf_msg->pose.pose.position;
        need_value[ekf] = false;
    }
    if(!res_src[ekf]){
        if (abs(ekf_msg->pose.pose.position.x) < 0.05 && abs(ekf_msg->pose.pose.position.y) < 0.05 ){
        res_src[ekf] = true;
        }
    }
}

void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amcl_msg){
     if(need_value[amcl]){
      amcl_pose = amcl_msg->pose.pose;
         if(amcl_msg->pose.pose.position.x !=amcl_msg->pose.pose.position.x || amcl_msg->pose.pose.position.y !=amcl_msg->pose.pose.position.y){
            cout << "amcl_msg nan!!!" << "\n";
            amcl_pose.position.x = 0;
            amcl_pose.position.y = 0;
         }
     }
     if(!res_src[amcl]){
         if (abs(amcl_msg->pose.pose.position.x) < 0.1 && abs(amcl_msg->pose.pose.position.y) < 0.1 ){
         res_src[amcl] = true;
         }
     }
}
double getYawOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point){
    geometry_msgs::Quaternion ori = curr_pose.orientation;
    double s_roll,s_pitch,s_yaw;
    tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(s_roll, s_pitch, s_yaw);
    if (s_yaw != s_yaw){
        cout << "start yaw was nan, set to 0";
        s_yaw = 0;
    }
    double yaw;
    cout << " ziel y " << goal_point.y << " ziel x " << goal_point.x <<" aktuell y " << curr_pose.position.y << " aktuell x " << curr_pose.position.x <<endl;
    yaw = atan2(goal_point.y- curr_pose.position.y ,goal_point.x- curr_pose.position.x);
    if (yaw != yaw ){
        cout << "yaw was nan, set to 0 " << yaw;
        cout << goal_point.y << curr_pose.position.y << goal_point.x  << curr_pose.position.x << endl;
        yaw = 0;}
    cout << "calculate yaw offset" << s_yaw << " ziel richtung: " << yaw << " diff: " << yaw - s_yaw << endl;
    return yaw - s_yaw; //rotiere um yaw = abweichung v. X-Koordinate + offset zur x-koordinate
}
double getXOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point){
    double d_x, d_y;
    d_x =  goal_point.x - curr_pose.position.x;
    d_y =  goal_point.y - curr_pose.position.y;
    return sqrt(pow(d_x,2)+pow(d_y,2));
}


void driveToPoint(geometry_msgs::Point goal_point, ros::Publisher& move_base){
    double x_diff,yaw_diff;
    bool cont = true;
    while(cont){
        ros::spinOnce();
        yaw_diff = getYawOffset(amcl_pose,goal_point);
        x_diff = getXOffset(amcl_pose,goal_point);
        if (abs(yaw_diff) > theta_yaw && x_diff > theta_x){
            cout << "start rotating about: " << yaw_diff << endl;
            rotate(move_base,yaw_diff*0.95);
        }else cont = false;
        if(x_diff > theta_x){
            cout << "start x-moving: " << x_diff << endl;
            drive(move_base,x_diff*0.5,speed);
            if (!cont){cont = true;}
        }
        //cont = false;
    }
    ros::spinOnce();
    cout << amcl_pose.orientation;
    cout << "x-offset: " << x_diff << endl;

}

geometry_msgs::Point genPoint(double x, double y){
    geometry_msgs::Point target;
    target.x = x;
    target.y = y;
    target.z = 0;
    cout << "created targets" <<endl;
    return target;
}


void resAllSources(ros::Rate* loop_rate,ros::NodeHandle* n){
    constexpr int odom = 0;
    constexpr int ekf = 1;
    constexpr int amcl = 2;
    res_src[odom] = res_src[ekf] = res_src[amcl] = false;
    int c = 0;
    ros::Publisher reset_ekf = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose",1000);
    ros::Publisher reset_amcl = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);
    while(!(res_src[odom] &&  res_src[ekf] && res_src[amcl])){
        resetImuOdom();
        resetEKF(&reset_ekf);
        resetAMCL(&reset_amcl);
        ros::spinOnce();
        loop_rate->sleep();
        c++;
        if (c > 20) break;
    }
    cout << "All sources reset; count of send res_msgs: " << c << endl;
}



int main(int argc,char **argv){
    ros::init(argc, argv, "drive_to_points");
    ros::NodeHandle n("~");
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",1000,odomCallback);
    ros::Subscriber ekf_sub = n.subscribe<nav_msgs::Odometry>("/ekf",1000,ekfCallback);
    ros::Subscriber amcl_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1000,amclCallback);
    ros::Publisher move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    bool cont = true;
    ros::Rate loop_rate(10);
    std::ofstream save_values;
    need_value[amcl];
    save_values.open("/home/husarion/husarion_ws/src/Beleg/evaluation.csv",std::ofstream::app);

    if (getBoolInput("Use predefined Path", true)) {
        resAllSources(&loop_rate,&n);

        // put some points to drive to
        need_value[amcl] = true;
        std::queue<geometry_msgs::Point> targets;
        targets.push(genPoint(1,1));
        /*targets.push(genPoint(1, 0));
        targets.push(genPoint(1.5, 0.5));
        targets.push(genPoint(-0.5, 1.5));
        targets.push(genPoint(0.25, -0.5));
        targets.push(genPoint(0, 0));*/
        while (!targets.empty()) {
            geometry_msgs::Point nextTarget = targets.front();
            targets.pop();
            driveToPoint(nextTarget,move_base);
            need_value[odom] = true;
            cout << "finale Position erreicht: " << amcl_pose << "\n";
            while (need_value[odom]){
                ros::spinOnce();
                loop_rate.sleep();
            }
            cout << " EKF-Position: " << odom_pose << "\n";
            std::cout << "Press Enter to continue..." << std::flush;
            std::time_t result = std::time(nullptr);
            save_values << nextTarget.x << "," << nextTarget.y << "," << amcl_pose.position.x << " ," << amcl_pose.position.y << "," << odom_pose.x << "," << odom_pose.y << "," << std::asctime(std::localtime(&result));
            std::cin.get();
        }
    }
    else {
        resAllSources(&loop_rate,&n);

        // wait for user to input coordinates to drive to
        do {
            double x = getDoubleInput("Next target x value");
            double y = getDoubleInput("Next target y value");

            need_value[amcl] = true;
            driveToPoint(genPoint(x,y),move_base);
            need_value[odom] = true;
            cout << "finale Position erreicht, x:" << amcl_pose.position.x << " y: " << amcl_pose.position.y << "\n";
            std::time_t result = std::time(nullptr);
            save_values << x << "," << y << "," << amcl_pose.position.x << " ," << amcl_pose.position.y << "," << odom_pose.x << "," << odom_pose.y << "," << std::asctime(std::localtime(&result));
            while (need_value[odom]){
                ros::spinOnce();
                loop_rate.sleep();
            }
            cout << " EKF-Position: " << odom_pose << "\n";
        } while (getBoolInput("Continue", true));
    }
    save_values << flush;
    save_values.close();

    exit(0);

}