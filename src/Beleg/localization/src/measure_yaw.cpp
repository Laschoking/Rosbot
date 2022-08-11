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
#include <geometry_msgs/Point.h>
#include "helpers.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <array>

bool get_odom_pose = false;
bool need_value[3] =  {false,false,false}; // odom, imu, gyro
const int odom = 0;
const int imu = 1;
const int gyro = 2;
geometry_msgs::Point odom_pos;
double values[2][2] = {{0,0},{0,0}};
std::ofstream save_values;
double angle = 180;
double sub_x;
double sub_y;
ros::Publisher* move_base;
enum States {setup,get_old_values,get_new_values};
States state = setup;
bool new_imu_data = false;
double angular_velocity = 0.5;
boost::shared_ptr<const sensor_msgs::Imu> old_imu = nullptr;
int ooo_imu = 0;
int c = 0;
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
        manage_states(yaw,odom);
	cout << "yaw_odom " << yaw << "\n";
        }
    if (get_odom_pose && odom_msg->pose.pose.position.x != 0){
        odom_pos = odom_msg->pose.pose.position;
        //cout << "odom pos x,y " << odom_pos.x  << " " << odom_pos.y << "\n";
        get_odom_pose = false;
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
            if(getBoolInput("starte Messung",true)){
                resetImuOdom();
                integratedImuYaw = 0.;
                need_value[odom] = true;
                need_value[imu] = true;
                state = get_old_values;
                break;
            }else{
		    save_values.close();
		    exit(0);}
        case get_old_values: //als vergleichswert (speziell IMU ist nach zurücksetzen nicht auf 0.0)
            //cout << source << " :alt:  " << yaw_to_degree(yaw) << endl;
            if (yaw < 0){
                yaw += 2*M_PI;
                if (yaw < 0){
                throw "wrong yaw value";
                cout << yaw;
                }
            }
            values[0][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                old_imu = nullptr;
                need_value[gyro] = true;
                rotate(move_base,angle,angular_velocity); // anti-clockwise = positive yaw
                need_value[gyro] = false;
                need_value[imu] = true;
                need_value[odom] = true;
                state = get_new_values;
            }
            break;
        case get_new_values:
            //cout << source << " :neu:  " << yaw_to_degree(yaw) << endl;
            /*if (yaw < 0){
                yaw += 2*M_PI;
                if (yaw < 0){
                throw "wrong yaw value";
                cout << yaw;
                }
            }*/
            //cout << source << " :neu:  " << yaw_to_degree(yaw) << endl;
            values[1][source] = yaw;
            need_value[source] = false;
            if (!need_value[odom] && !need_value[imu]){
                drive(move_base,0.3,0.2);
                get_odom_pose = true;
                cout << "Differenz von Rotation & Bewegung: " << endl;
                while (get_odom_pose){
                    ros::spinOnce();
                    ros::Duration(0.01).sleep();
                    }
                double diff_yaw[3];
                for (int i = 0; i < 2; i++){
                    double tmp_yaw = values[1][i] - values[0][i];
                    if (tmp_yaw > 2*M_PI){
                        tmp_yaw = fmod(tmp_yaw,2*M_PI);
                        }
                    else if (tmp_yaw > M_PI){
                        tmp_yaw = 2*M_PI - tmp_yaw;
                    }else if (tmp_yaw < -M_PI){
                        tmp_yaw += 2*M_PI;
                    }
                    diff_yaw[i] = tmp_yaw;
                }
                if (integratedImuYaw < -M_PI){
                    integratedImuYaw += 2*M_PI;
                }else if(integratedImuYaw > 2*M_PI){
                    integratedImuYaw = fmod(integratedImuYaw, 2*M_PI);
                }
                c++;
		diff_yaw[2] = integratedImuYaw;
                double x = getDoubleInput("x-wert gemessen",0);
                //x -= getDoubleInput("Subtrahiere Distanz",0); //falls roboter zu nahe an mess linie ist muss von anderer Linie gemessen werden
                // naechste linie : 0.164
                x -= sub_x;
		double y = getDoubleInput("y-wert gemessen",0);
                //y -= getDoubleInput("Subtrahiere Distanz",0); //falls roboter zu nahe an mess linie ist muss von anderer Linie gemessen werden
                y -= sub_y;
		double meas_yaw = atan2(y,x);
                if (meas_yaw > M_PI){
                    meas_yaw = 2*M_PI - meas_yaw;}
                std::time_t result = std::time(nullptr);//
                cout << "Messung Nr. " << c << " [yaw, ang_speed, meas_yaw, Odom,Imu, Imu_integrated, meas_x, meas_y , odom_x, odom_y] : \n";
		cout << " [" << yaw_to_degree(angle) << "," << angular_velocity << "," << yaw_to_degree(meas_yaw) << "," << yaw_to_degree(diff_yaw[0])  << "," << yaw_to_degree(diff_yaw[1]) << "," << yaw_to_degree(diff_yaw[2]) << ","<< x << "," << y << ","  << odom_pos.x << "," << odom_pos.y << "]\n";
                save_values << angle << "," << angular_velocity << "," << meas_yaw << "," << diff_yaw[0] << "," << diff_yaw[1] << ","  << diff_yaw[2] << "," << x << "," << y << "," <<  odom_pos.x << "," << odom_pos.y << "," <<  std::asctime(std::localtime(&result));
                save_values << flush;
		cout << "Werte in CSV-Datei gespeichert" << endl;
                if(getBoolInput("Zurueckfahren",true)){
                    drive(move_base,0.3,-0.2);
                    rotate(move_base,-angle,0.75);
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
    ros::Subscriber odom_wheel_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",1,odomCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu",5,imuCallback);
    auto tmp = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    move_base = &tmp;
    if (getBoolInput("Starte Rotationsmessung",true)){
        save_values.open("/home/husarion/husarion_ws/src/Beleg/yaw.csv",std::ofstream::app);
        state = setup;
        angle = degree_to_yaw(getDoubleInput("Winkelangabe in Grad",angle));
        angular_velocity = getDoubleInput("Angular Velocity",angular_velocity);
        sub_x = getDoubleInput("immer x-Wert von Messung abziehen",0);
        sub_y = getDoubleInput("immer y-Wert von Messung abziehen",0);
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
