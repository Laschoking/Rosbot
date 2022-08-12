#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <sstream>
#include <chrono>
#include "drive_to_points.h"
#include "helpers.h"
#include "monitor_process.h"
#include "sqlite_lib.h"
#include <tf/transform_datatypes.h>
#include "driver.h"
#include <cmath>
#include <array>
#include <thread>
#include <sqlite3.h>
#include <std_msgs/UInt32.h>
#include <unistd.h>


struct {
    double proc_x_mean;
    double proc_yaw_mean;
    double proc_x_var;
    double proc_yaw_var;
    double odom_x_var;
    double odom_yaw_var;
    double imu_yaw_var;
    double imu_int_yaw_var;}
    meas_const;

bool need_value[3];
bool is_res[3];
constexpr int odom = 0;
constexpr int ekf = 1;
constexpr int amcl = 2;
const double theta_yaw = 0.2; //radians
const double theta_x = 0.07;  //meters
geometry_msgs::Point odom_pose;
geometry_msgs::Point ekf_pose;
geometry_msgs::Pose amcl_pose;
unsigned int amcl_seq = 0;
bool new_amcl_data = false;

void odomCallback(boost::shared_ptr< const nav_msgs::Odometry> odom_msg){
    if(need_value[odom]){
        odom_pose = odom_msg->pose.pose.position;
        need_value[odom] = false;
    }
    if(!is_res[odom]){
        if (abs(odom_msg->pose.pose.position.x) < 0.05 && abs(odom_msg->pose.pose.position.y) < 0.05 ){
        is_res[odom] = true;
        }
    }
}


void ekfCallback(boost::shared_ptr< const nav_msgs::Odometry> ekf_msg){
    if(need_value[ekf]){
        ekf_pose = ekf_msg->pose.pose.position;
        need_value[ekf] = false;
    }
    if(!is_res[ekf]){
        if (abs(ekf_msg->pose.pose.position.x) < 0.05 && abs(ekf_msg->pose.pose.position.y) < 0.05 ){
            is_res[ekf] = true;
        }
    }
}

void amclCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> amcl_msg){
     if(need_value[amcl]){
        if((unsigned int) amcl_msg->header.seq > amcl_seq){
            amcl_seq = (unsigned int) amcl_msg->header.seq;
            new_amcl_data = true;
        }else std::cout << "no update of seq: " << amcl_seq << " " << amcl_msg->header.seq << "\n";
        amcl_pose = amcl_msg->pose.pose;
        if(amcl_msg->pose.pose.position.x !=amcl_msg->pose.pose.position.x || amcl_msg->pose.pose.position.y !=amcl_msg->pose.pose.position.y){
            std::cout << "amcl_msg nan!!!" << "\n";
            amcl_pose.position.x = 0;
            amcl_pose.position.y = 0;
        }
     }
     if(!is_res[amcl]){
         if (abs(amcl_msg->pose.pose.position.x) < 0.1 && abs(amcl_msg->pose.pose.position.y) < 0.1 ){is_res[amcl] = true;
         }
     }
}

geometry_msgs::Point genPoint(double x, double y){
    geometry_msgs::Point target;
    target.x = x;
    target.y = y;
    target.z = 0;
    return target;
}


void resAllSources(ros::Rate* loop_rate,ros::NodeHandle* n){
    constexpr int odom = 0;
    constexpr int ekf = 1;
    constexpr int amcl = 2;
    is_res[odom] = is_res[ekf] = is_res[amcl] = false;
    int c = 0;
    bool cont = true;
    ros::Publisher reset_ekf = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose",1000);
    ros::Publisher reset_amcl = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);
    std::cout << is_res[odom] << is_res[ekf] << is_res[amcl] << std::endl;
    while(cont && !(is_res[odom] &&  is_res[ekf] && is_res[amcl])){
        std::cout << "test1" <<std::endl;
        resetImuOdom();
        std::cout << "test2" <<std::endl;
        resetEKF(&reset_ekf);
        std::cout << "test3" <<std::endl;
        resetAMCL(&reset_amcl);
        std::cout << "test4" <<std::endl;
        ros::spinOnce();
        loop_rate->sleep();
        c++;
        std::cout << c << " " << amcl_pose.position.x << " " << amcl_pose.position.y <<std::endl;
        if (c > 20) {
            std::cout << "could not reset all sensors! ";
            std::cout << "ekf "<< ekf_pose.x << " " << ekf_pose.y ;
            std::cout << "odom "<< odom_pose.x << " " << odom_pose.y ;
            std::cout << "amcl "<< amcl_pose.position.x << " " << amcl_pose.position.y ;
            std::cout << std::flush;
            cont = false;}
    }
    std::cout << "All sources reset; count of send res_msgs: " << c << std::endl;
}


//evaluation -> Key: Durchlauf (da mehrere Punkte), Parameter-verweis o.ä., Soll-Position , gemessene Position  (?), AMCL-Position, EKF , ODOM, Dauer, AVG_CPU, AVG_MEM

monitor_results* driveToPoint(geometry_msgs::Point* goal_point, ros::Publisher* move_base,sqlite3* db, int iteration, int goal_nr,const double speed, const double ang_vel){
    double x_diff,yaw_diff;
    bool cont = true;
    std::string file = "/home/husarion/husarion_ws/src/Beleg/localization/tmp_res.txt";
    bool monitor_succ = true;
    std::unique_ptr<std::thread> monitor_ptr;
    ros::Time t_begin = ros::Time::now();
    try{
        monitor_ptr = std::unique_ptr<std::thread>(new std::thread(monitor_process,&file,&t_begin));

        }catch(int i){
            std::cout << "could not attach to current process \n";
            monitor_succ = false;
        }
    while(cont){
        ros::spinOnce();
        ros::Duration(0.05).sleep();        //sleep for amcl update
        //last amcl_seq = x
        new_amcl_data = false;
        //in rotate and drive spinOnce(), so amcl etc. gets updated
        yaw_diff = getYawOffset(&amcl_pose,goal_point);
        x_diff = getXOffset(&amcl_pose,goal_point);
        double s_roll,s_pitch,s_yaw;
                tf::Matrix3x3(tf::Quaternion{amcl_pose.orientation.x, amcl_pose.orientation.y, amcl_pose.orientation.z, amcl_pose.orientation.w}).getRPY(s_roll, s_pitch, s_yaw);
        std::cout << "\n amcl_seq_nr:" << amcl_seq << " x: " <<  amcl_pose.position.x << " y: "<< amcl_pose.position.y << " own yaw "<< s_yaw <<  " X_DIFF: " << x_diff <<" yaw_diff: "  << yaw_diff << "\n";

        //std::cout << "own orientation (yaw): " << s_yaw <<"\n";
        if (abs(yaw_diff) > theta_yaw && x_diff > theta_x){
            std::cout << " -> rotate for "<< yaw_diff << "\n";
            rotate(move_base,yaw_diff,ang_vel,meas_const.proc_yaw_mean,&new_amcl_data);
            if (new_amcl_data) {
                std::cout << "received new amcl_information while rotating:\n";
                std::cout << "amcl_seq_nr:" << amcl_seq << " x: " <<  amcl_pose.position.x << " y: "<< amcl_pose.position.y << "\n";
                continue;
            } //stopped the rotation -> cont. rotatation before translation
        }
        if(x_diff > theta_x){
            std::cout << " -> drive for " << x_diff << "\n";
            drive(move_base,x_diff,speed,meas_const.proc_x_mean,&new_amcl_data);
            if (new_amcl_data) {
                std::cout << "received new amcl_information while driving\n";
                std::cout << "amcl_seq_nr:" << amcl_seq << " x: " <<  amcl_pose.position.x << " y: "<< amcl_pose.position.y  << "\n";
                continue; //stopped the rotation -> cont. rotatation before translation
            }
        }
        cont = false;
        /*
        //in csae ROSBOT thinks it is at goal -> update manually
        new_amcl_data = false;
        cont = false; //try one more time to get manual position update
        int k = requestAmclUpdate();
        while(!new_amcl_data){
            ros::spinOnce();
            ros::Duration(0.25).sleep();
            std::cout << "wait for AMCL Update" << std::endl;
        }
        yaw_diff = getYawOffset(&amcl_pose,goal_point);
        x_diff = getXOffset(&amcl_pose,goal_point);
        if (x_diff > theta_x){
            std::cout << "MANUAL UPDATE (move):: amcl_seq_nr: " << amcl_seq << " x: " <<  amcl_pose.position.x << " y: "<< amcl_pose.position.y << "\n";
            cont = true;
        }else{
            std::cout << "MANUAL UPDATE (dont move):: amcl_seq_nr: " << amcl_seq << " x: " <<  amcl_pose.position.x << " y: "<< amcl_pose.position.y << "\n";
        }*/

    }
    monitor_results* res = NULL;
    if (monitor_succ){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        res = end_monitor(monitor_ptr,&file,&t_begin,db, iteration, goal_nr);
    }
    ros::spinOnce();
    return res;
}




int main(int argc,char **argv){
    ros::init(argc, argv, "drive_to_points");
    ros::NodeHandle n("~");
    const double speed = getDoubleInput("speed",0.6);
    const double ang_vel = getDoubleInput("ang_vel",0.5);

    sqlite3* db;
    std::string db_file = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db";
    if(sqlite3_open(db_file.c_str(),&db) != SQLITE_OK){
          printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
    }
    if(speed != 0.1 && speed != 0.2 && speed != 0.4 && speed != 0.6 && speed != 0.8){
        std::cout << "invalid argument, given speed doesnt fit to measurement!" << speed << "\n";
    }else{ //query database for correct value
      std::string sql = "SELECT proc_x_mean FROM acc_eval WHERE speed = " + std::to_string(speed) + " ;";
      meas_const.proc_x_mean = 0; //getSQLiteOut(db,&sql);
      sql = "SELECT odom_x_var FROM acc_eval WHERE speed = "+ std::to_string(speed) + ";";
      meas_const.odom_x_var = getSQLiteOut(db,&sql);
   }
   if(ang_vel != 0.25 && ang_vel != 0.5 && ang_vel != 0.75 && ang_vel != 1){
           std::cout << "invalid argument, given speed doesnt fit to measurement!" << ang_vel << "\n";
       }else{ //query database for correct value
         std::string sql = "SELECT proc_mean FROM yaw_eval WHERE ang_speed = " + std::to_string(ang_vel) + " ;";
         meas_const.proc_yaw_mean = 0; //getSQLiteOut(db,&sql);
         sql = "SELECT odom_var FROM yaw_eval WHERE ang_speed = " + std::to_string(ang_vel) + " ;";
         meas_const.odom_yaw_var = getSQLiteOut(db,&sql);
         sql = "SELECT imu_var FROM yaw_eval WHERE ang_speed = " + std::to_string(ang_vel) + " ;";
         meas_const.imu_yaw_var = getSQLiteOut(db,&sql);
         sql = "SELECT imu_int_var FROM yaw_eval WHERE ang_speed = " + std::to_string(ang_vel) + " ;";
         meas_const.imu_int_yaw_var = getSQLiteOut(db,&sql);
      }
   std::string sql = "SELECT iteration FROM evaluation ORDER BY iteration DESC LIMIT 1;";
   double tmp = getSQLiteOut(db,&sql);
   int iteration;
   if(tmp != tmp){
         iteration = 0;
   }else{
       iteration = (int)tmp;
       iteration++;

   }
    std::cout << "iteration in evaluation db: "<< iteration << "\n";
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom/wheel",100,odomCallback);
    ros::Subscriber ekf_sub = n.subscribe<nav_msgs::Odometry>("/ekf",1,ekfCallback);
    ros::Subscriber amcl_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1,amclCallback);
    ros::Publisher move_base = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    bool cont = true;
    ros::Rate loop_rate(10);
    std::ofstream save_values;
    save_values.open("/home/husarion/husarion_ws/src/Beleg/evaluation.csv",std::ofstream::app);
    resAllSources(&loop_rate,&n);
    need_value[amcl] = true;

    if (getBoolInput("Use predefined Path", true)) {
        // put some points to drive to
        std::queue<geometry_msgs::Point> targets;
        targets.push(genPoint(0.5, 1.5));
        targets.push(genPoint(1.5, 0.5));
        targets.push(genPoint(2, -1));
        targets.push(genPoint(1, 0));
        targets.push(genPoint(0.5, -0.5));
        targets.push(genPoint(0,0));
        int goal_nr = 0;
        while (!targets.empty()) {
            geometry_msgs::Point nextTarget = targets.front();
            int nr_amcl_updates = amcl_seq;
            targets.pop();
            ros::Time begin = ros::Time::now();
            monitor_results* res = driveToPoint(&nextTarget,&move_base,db,iteration,goal_nr, speed,ang_vel);
            nr_amcl_updates = amcl_seq - nr_amcl_updates;
            ros::Duration dur = ros::Time::now() - begin;
            need_value[odom] = true;
            need_value[ekf] = true;
            while (need_value[odom] || need_value[ekf]){
                ros::spinOnce();
                loop_rate.sleep();
            } // Warte auf neueste odom & ekf werte
            double meas_x = getDoubleInput("x-distance to target");
            double meas_y = getDoubleInput("y-distance to target");
            //substract x-offset (introduced with each measurement
            meas_x -= 0.123;
            std::cout << "Position erreicht [Ziel, amcl, ekf, odom]" <<"\n";
            std::cout << "[" << nextTarget.x << "," <<  amcl_pose.position.x << "," << ekf_pose.x << "," << odom_pose.x << "]\n";
            std::cout << "[" << nextTarget.y << "," << amcl_pose.position.y << "," << ekf_pose.y << "," << odom_pose.y << "]\n";
            std::cout << "Dauer der Operation: " << dur << "\n";
            std::cout << std::flush;
            std::time_t t_begin;
            time(&t_begin);
            std::string loc_time = asctime(localtime(&t_begin));
            loc_time.pop_back(); // remove new line that asctime insert by default
            std::string sql = "INSERT INTO evaluation VALUES (" + std::to_string(iteration) + "," + std::to_string(goal_nr)
                + "," +  std::to_string(nextTarget.x) + "," + std::to_string(nextTarget.y) + "," + std::to_string(meas_x) + "," + std::to_string(meas_y)
                + "," + std::to_string(amcl_pose.position.x) + "," + std::to_string(amcl_pose.position.y) + "," + std::to_string(ekf_pose.x)
                + "," + std::to_string(ekf_pose.y) + "," + std::to_string(odom_pose.x) + "," + std::to_string(odom_pose.y) + ",\""+ loc_time
                + "\"," + std::to_string(dur.toSec()) + "," + std::to_string(res->cpu_avg) + "," + std::to_string(res->mem_avg)
                + "," + std::to_string(speed) +  "," + std::to_string(ang_vel) + "," +  std::to_string(nr_amcl_updates) + ");";
            //std::cout << sql;
            if (!insertSQLite(db,&sql)) std::cout << "Error when trying to insert data!\n";

            save_values << nextTarget.x << "," << nextTarget.y << "," << amcl_pose.position.x << " ," << amcl_pose.position.y << "," << ekf_pose.x << " ," << ekf_pose.y << "," << odom_pose.x << "," << odom_pose.y << "," << std::asctime(std::localtime(&t_begin));
            goal_nr ++;
            std::cout << "\nPress Enter to continue..." << std::flush;
            std::cin.get();
        }
    }
    else {
        int goal_nr = 0;
        // wait for user to input coordinates to drive to
        do {
            double x = getDoubleInput("Next target x value");
            double y = getDoubleInput("Next target y value");
            ros::Time begin = ros::Time::now();
            geometry_msgs::Point p = genPoint(x,y);
            monitor_results* res = driveToPoint(&p,&move_base,db,iteration,goal_nr,speed, ang_vel);
            ros::Duration dur = ros::Time::now() - begin;

            need_value[odom] = true;
            need_value[ekf] = true;
            while (need_value[odom] || need_value[ekf]){
                ros::spinOnce();
                loop_rate.sleep();
            } // Warte auf neueste odom & ekf werte
            std::cout << "Position erreicht [Ziel, amcl, ekf, odom]" <<"\n";
            std::cout << "[" << x << "," <<  amcl_pose.position.x << "," << ekf_pose.x << "," << odom_pose.x << "]\n";
            std::cout << "[" << y << "," << amcl_pose.position.y << "," << ekf_pose.y << "," << odom_pose.y << "]\n";
            std::cout << " cpu-usage: " << res->cpu_avg << " mem_usage: " << res->mem_avg << std::endl;
            std::cout << "Dauer der Operation: " << dur << "  dauer vom thread gemessen:" << res->duration << " seconds\n";
            std::cout << std::flush;

            std::time_t result = std::time(nullptr);
            save_values << x << "," << y << "," << amcl_pose.position.x << " ," << amcl_pose.position.y << ","<< ekf_pose.x << " ," << ekf_pose.y << "," << odom_pose.x << "," << odom_pose.y << "," << std::asctime(std::localtime(&result));
            goal_nr++;
        } while (getBoolInput("Continue", true));
    }
    save_values << std::flush;
    save_values.close();
    sqlite3_close(db);
    exit(0);

}
