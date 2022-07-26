#include <ros/ros.h>
#include <ctime>
#include "measure_yaw.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include "helpers.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <array>

//void manage_states(double yaw, double source);
//void rotate_vehicle(double angle, double distance);
bool need_value[3] =  {false,false,false}; // odom, imu, gyro
const int odom = 0;
const int imu = 1;
const int gyro = 2;

double values[2][2] = {{0,0},{0,0}};
std::ofstream save_values;
double angle = 180;
ros::Publisher move_base;
enum States {setup,get_old_values,get_new_values};
States state = setup;
bool new_imu_data = false;
boost::shared_ptr<const sensor_msgs::Imu> old_imu = nullptr;
int ooo_imu = 0;
double integratedImuYaw;
using namespace std;

void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom_msg){
    if (need_value[odom]){
        if(state == get_old_values && odom_msg->pose.pose.position.x != 0){
            //cout << "waiting for reset of odom"<<endl;
            return;
         }
        geometry_msgs::Quaternion ori = odom_msg->pose.pose.orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        manage_states(yaw,odom); //TODO kann yaw auch negative Werte annehmen??
        }
}

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    if(need_value[gyro]){
        integrateImu(imu_msg);
    }
    if (need_value[imu]){
        geometry_msgs::Quaternion ori = imu_msg->orientation;
        double roll,pitch,yaw;
        tf::Matrix3x3(tf::Quaternion{ori.x, ori.y, ori.z, ori.w}).getRPY(roll, pitch, yaw);
        manage_states(yaw,imu);
        }
}

void integrateImu(boost::shared_ptr<const sensor_msgs::Imu> imu_msg){
    if (old_imu == nullptr){
        old_imu = imu_msg;
        //cout << "set old_imu value " << endl;
        return;
    }
    if(imu_msg->header.seq < old_imu->header.seq){
        ooo_imu ++;
        cout << "out of order IMU message, count:  " << ooo_imu << endl;
        cout << "old: " << old_imu->header.seq << " new: " << imu_msg->header.seq << endl;
        return;
    }
    //cout << "new  seq & sec& nsec : "<< imu_msg->header.seq << "  " << imu_msg->header.stamp.sec << "  " << imu_msg->header.stamp.nsec << endl;
    ros::Time t1 = imu_msg->header.stamp;
    ros::Time t2 =  old_imu->header.stamp;
    ros::Duration duration = t1 - t2;
    //cout << "duration: " << duration.toSec() << endl;
    double rel_yaw = old_imu->angular_velocity.z * duration.toSec();
    integratedImuYaw += rel_yaw;
    old_imu = imu_msg;
    //cout << "imu_ang_vel, rel_yaw: " << rel_yaw  << " abs_yaw: " << integratedImuYaw << endl;
}

void manage_states(double yaw, bool source){
    switch (state){
        case setup:
{           angle = getDoubleInput("Winkelangabe in Grad?",angle);
            resetImuOdom();
	        integratedImuYaw = 0.;
            need_value[odom] = true;
            need_value[imu] = true;
            state = get_old_values;
            break;}
        case get_old_values: //als vergleichswert (speziell IMU ist nach zurücksetzen nicht auf 0.0)
            //cout << source << " :alt:  " << yaw_to_degree(yaw) << endl;
            if (yaw < 0){
                yaw += 2*M_PI;
                if (yaw < 0){
                throw "wrong yaw value";
                cout << yaw;
                }
            }
            cout << source << " :alt:  " << yaw_to_degree(yaw) << endl;
            values[0][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                old_imu = nullptr;
                need_value[gyro] = true;
                rotate(move_base,degree_to_yaw(angle)); // anti-clockwise = positive yaw
                need_value[gyro] = false;
                drive(move_base,0.5,0.2);
                need_value[imu] = true;
                need_value[odom] = true;
                state = get_new_values;
            }
            break;
        case get_new_values:
            cout << source << " :neu:  " << yaw_to_degree(yaw) << endl;

            if (yaw < 0){
                yaw += 2*M_PI;
                if (yaw < 0){
                throw "wrong yaw value";
                cout << yaw;
                }
            }
            //cout << source << " :neu:  " << yaw_to_degree(yaw) << endl;
            values[1][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                cout << "Differenz von Rotation & Bewegung: " << endl;
                double diff_yaw[3];
                for (int i = 0; i < 2; i++){
                    double tmp_yaw = yaw_to_degree(values[1][i] - values[0][i]);
                    tmp_yaw = tmp_yaw > 360 ? fmod(tmp_yaw, 360) : tmp_yaw;
                    tmp_yaw += tmp_yaw < 0 ? 360 : 0;
                    diff_yaw[i] = tmp_yaw;
                }
                if (integratedImuYaw < 0){
                    integratedImuYaw += 2*M_PI;
                }else if(integratedImuYaw > 2*M_PI){
                    integratedImuYaw = fmod(integratedImuYaw, 2*M_PI);
                }
                diff_yaw[2] = yaw_to_degree(integratedImuYaw);
                if(angle < 0){
                    for (int i = 0; i < 3; i++){
                        diff_yaw[i] -= 360;
                    }
                }
                cout << "Odom_yaw in Grad: " << diff_yaw[0] << "\n";
                cout << "Imu_yaw in Grad: " << diff_yaw[1] << "\n";
                cout << "IMU_integrated in grad: " << diff_yaw[2] << endl;
                std::time_t result = std::time(nullptr);
                save_values << angle << "," << diff_yaw[0] << "," << diff_yaw[1] << ","  << diff_yaw[2] << "," <<  std::asctime(std::localtime(&result));
                cout << "Werte in CSV-Datei gespeichert" << endl;
                if(getBoolInput("Zurueckfahren + neue Messung?",true)){
                    rotate(move_base,M_PI);
                    drive(move_base,0.5,0.2);
                    state = setup;
                    manage_states(0,0);
                }else {
                    exit(0);
                 }
             }
            break;
        }
    }


int main(int argc,char **argv) {
    // initialize the ros systems
    ros::init(argc, argv, "measure_yaw");
    ros::NodeHandle n("~");
    ros::Subscriber odom_wheel_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",1000,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",1000,imuCallback);
    move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    if (getBoolInput("Starte Rotationsmessung?",true)){
        save_values.open("/home/husarion/husarion_ws/src/Beleg/yaw.csv",std::ofstream::app);
        state = setup;
        manage_states(0,0);
    }
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep(); // Don't forget this! *
    }
    save_values.close();
    exit(0);
}
