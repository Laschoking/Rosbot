#include <ros/ros.h>
#include <ctime>
#include "drive_to_points.h"
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <sstream>
#include <chrono>
#include "helpers.h"
#include "monitor_process.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <array>
#include <thread>
#include <sqlite3.h>
#include <unistd.h>

bool need_value[3];
bool res_src[3];
constexpr int odom = 0;
constexpr int ekf = 1;
constexpr int amcl = 2;
const double theta_yaw = 0.3; //radians
const double theta_x = 0.06;  //meters
geometry_msgs::Point odom_pose;
geometry_msgs::Point ekf_pose;
geometry_msgs::Pose amcl_pose;


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
            std::cout << "amcl_msg nan!!!" << "\n";
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
        std::cout << "start yaw was nan, set to 0";
        s_yaw = 0;
    }
    double yaw;
    //std::cout << " ziel y " << goal_point.y << " ziel x " << goal_point.x <<" aktuell y " << curr_pose.position.y << " aktuell x " << curr_pose.position.x <<std::std::endl;
    yaw = atan2(goal_point.y- curr_pose.position.y ,goal_point.x- curr_pose.position.x);
    if (yaw != yaw ){
        std::cout << "yaw was nan, set to 0 " << yaw;
        std::cout << goal_point.y << curr_pose.position.y << goal_point.x  << curr_pose.position.x << std::endl;
        yaw = 0;}
    //std::cout << "calculate yaw offset" << s_yaw << " ziel richtung: " << yaw << " diff: " << yaw - s_yaw << std::endl;
    return yaw - s_yaw; //rotiere um yaw = abweichung v. X-Koordinate + offset zur x-koordinate
}
double getXOffset(geometry_msgs::Pose curr_pose, geometry_msgs::Point goal_point){
    double d_x, d_y;
    d_x =  goal_point.x - curr_pose.position.x;
    d_y =  goal_point.y - curr_pose.position.y;
    return sqrt(pow(d_x,2)+pow(d_y,2));
}

//evaluation -> Key: Durchlauf (da mehrere Punkte), Parameter-verweis o.ä., Soll-Position , gemessene Position  (?), AMCL-Position, EKF , ODOM, Dauer, AVG_CPU, AVG_MEM

monitor_results* driveToPoint(geometry_msgs::Point goal_point, ros::Publisher& move_base,const double speed, const double ang_vel, const double proc_x_mean, const double proc_yaw_mean){
    double x_diff,yaw_diff;
    bool cont = true;
    pid_t pid = getpid();
    std::string file = "/home/husarion/husarion_ws/src/Beleg/localization/tmp_res.txt";
    bool monitor_succ = true;
    std::unique_ptr<std::thread> monitor_ptr;
    try{
        monitor_ptr = std::unique_ptr<std::thread>(new std::thread(monitor_process,&pid,&file));

        }catch(int i){
            std::cout << "could not attach to current process " << std::to_string(pid) << "\n";
            monitor_succ = false;
        }
    ros::Time begin = ros::Time::now();
    while(cont){
        ros::spinOnce();
        yaw_diff = getYawOffset(amcl_pose,goal_point);
        x_diff = getXOffset(amcl_pose,goal_point);
        if (abs(yaw_diff) > theta_yaw && x_diff > theta_x){
            //std::cout << "start rotating about: " << yaw_diff << std::endl;
            rotate(move_base,yaw_diff,ang_vel,proc_yaw_mean);
        }else cont = false;
        if(x_diff > theta_x){
            //std::cout << "start x-moving: " << x_diff << std::endl;
            if (x_diff < 0.1){
                drive(move_base,x_diff,speed,proc_x_mean); //amcl_update rate
            }else drive(move_base,x_diff*0.25,speed,proc_x_mean);
            if (!cont){cont = true;}
        }
        //cont = false;
    }
    monitor_results* res = NULL;
    if (monitor_succ){
        res = end_monitor(monitor_ptr,&file,&begin);
        res->pid = (int) pid;
    }

    ros::spinOnce();
    return res;
}

geometry_msgs::Point genPoint(double x, double y){
    geometry_msgs::Point target;
    target.x = x;
    target.y = y;
    target.z = 0;
    std::cout << "created targets" <<std::endl;
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
    std::cout << "All sources reset; count of send res_msgs: " << c << std::endl;
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
    resAllSources(&loop_rate,&n);
    need_value[amcl] = true;
    const double speed = getDoubleInput("speed",0.6);
    const double ang_vel = getDoubleInput("ang_vel",0.5);
    sqlite3* db;
    std::string db_file = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db";
    if(sqlite3_open(db_file.c_str(),&db) != SQLITE_OK){
          printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
    }
    double proc_x_mean = 0;
    double proc_yaw_mean = 0;
    if(speed != 0.1 && speed != 0.2 && speed != 0.4 && speed != 0.6 && speed != 0.8){
        std::cout << "invalid argument, given speed doesnt fit to measurement!" << speed << "\n";
    }else{ //query database for correct value
      std::string sql = "SELECT proc_x_mean FROM acc_eval WHERE speed = " + std::to_string(speed) + " ;";
      sqlite3_stmt* stmt;
      std::cout << sql << "\n";
      if(sqlite3_prepare_v2(db,sql.c_str(),-1, &stmt, NULL) != SQLITE_OK){
        printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        save_values.close();
        exit(0);

      }
      while(sqlite3_step(stmt) == SQLITE_ROW){
            proc_x_mean = (double) sqlite3_column_double(stmt,0);
            std::cout << "einbeziehung der Abweichung: " << sqlite3_column_double(stmt,0) <<"\n";
      }
      sqlite3_finalize(stmt);
   }
   if(ang_vel != 0.25 && ang_vel != 0.5 && ang_vel != 0.75 && ang_vel != 1){
           std::cout << "invalid argument, given speed doesnt fit to measurement!" << ang_vel << "\n";
       }else{ //query database for correct value
         std::string sql = "SELECT proc_mean FROM yaw_eval WHERE ang_speed = " + std::to_string(ang_vel) + " ;";
         sqlite3_stmt* stmt;
         std::cout << sql << "\n";
         if(sqlite3_prepare_v2(db,sql.c_str(),-1, &stmt, NULL) != SQLITE_OK){
           printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
           sqlite3_finalize(stmt);
           sqlite3_close(db);
           save_values.close();
           exit(0);
         }
         while(sqlite3_step(stmt) == SQLITE_ROW){
               proc_yaw_mean = (double) sqlite3_column_double(stmt,0);
               std::cout << "einbeziehung der Abweichung: " << sqlite3_column_double(stmt,0) <<"\n";
         }
         sqlite3_finalize(stmt);
      }

    if (getBoolInput("Use predefined Path", true)) {

        // put some points to drive to
        std::queue<geometry_msgs::Point> targets;
        targets.push(genPoint(1,1));
        targets.push(genPoint(2.5, 1.5));
        targets.push(genPoint(0.5, -1));
        targets.push(genPoint(1, 0));
        while (!targets.empty()) {
            geometry_msgs::Point nextTarget = targets.front();
            targets.pop();
            ros::Time begin = ros::Time::now();
            monitor_results* res = driveToPoint(nextTarget,move_base,speed,ang_vel,proc_x_mean, proc_yaw_mean);
            ros::Duration dur = ros::Time::now() - begin;
            need_value[odom] = true;
            need_value[ekf] = true;
            while (need_value[odom] || need_value[ekf]){
                ros::spinOnce();
                loop_rate.sleep();
            } // Warte auf neueste odom & ekf werte
            std::cout << "Position erreicht [Ziel, amcl, ekf, odom]" <<"\n";
            std::cout << "[" << nextTarget.x << "," <<  amcl_pose.position.x << "," << ekf_pose.x << "," << odom_pose.x << "]\n";
            std::cout << "[" << nextTarget.y << "," << amcl_pose.position.y << "," << ekf_pose.y << "," << odom_pose.y << "]\n";
            std::cout << "Dauer der Operation: " << dur <<  "dauer vom thread gemessen:" << res->duration << "\n";
            std::cout << std::flush;
            std::time_t result = std::time(nullptr);
            save_values << nextTarget.x << "," << nextTarget.y << "," << amcl_pose.position.x << " ," << amcl_pose.position.y << "," << ekf_pose.x << " ," << ekf_pose.y << "," << odom_pose.x << "," << odom_pose.y << "," << std::asctime(std::localtime(&result));
            std::cout << "Press Enter to continue..." << std::flush;
            std::cin.get();
        }
    }
    else {

        // wait for user to input coordinates to drive to
        do {
            double x = getDoubleInput("Next target x value");
            double y = getDoubleInput("Next target y value");
            ros::Time begin = ros::Time::now();
            monitor_results* res = driveToPoint(genPoint(x,y),move_base,speed, ang_vel,proc_x_mean, proc_yaw_mean);
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
            std::cout << "prozess_id: " << res->pid << " cpu-usage: " << res->cpu_avg << " mem_usage: " << res->mem_avg << std::endl;
            std::cout << "Dauer der Operation: " << dur << "  dauer vom thread gemessen:" << res->duration << " seconds\n";
            std::cout << "thread_id" << std::this_thread::get_id() << "\n";
            std::cout << std::flush;

            std::time_t result = std::time(nullptr);
            save_values << x << "," << y << "," << amcl_pose.position.x << " ," << amcl_pose.position.y << ","<< ekf_pose.x << " ," << ekf_pose.y << "," << odom_pose.x << "," << odom_pose.y << "," << std::asctime(std::localtime(&result));

        } while (getBoolInput("Continue", true));
    }
    save_values << std::flush;
    save_values.close();
    sqlite3_close(db);
    exit(0);

}