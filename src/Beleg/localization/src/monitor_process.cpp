//
// Created by kotname on 04.08.22.
//
#include "monitor_process.h"
#include "sqlite_lib.h"
#include "sys/types.h"
#include "sys/sysinfo.h"
#include <unistd.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <atomic>
#include <thread>
#include <time.h>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

struct sysinfo memInfo;




//take timestamps seperate because top/ps time is only accurate to the second (not millisecond)
std::vector<double> time_stamps;
//multi-threading lock
std::atomic<bool> stop_monitor(false);
//monitor: amcl, robot_localization, evaluation node
//monitor resources of the all.launch file (call by roslaunch localization all.launch
// it launches multiple processes: amcl, ekf_localization, map_server, msgs_conversation,python3, rplidarNode,2* stat_transform
// order of the return is fixed alphabetically descending
void monitor_process(std::string* file,ros::Time* t_begin) {
    std::fstream f_in(*file,std::ios::out | std::ios::trunc); //clear old file
    f_in.close();
    while (!stop_monitor.load()) {
        ros::Time test = ros::Time::now();
        std::string s = "pgrep roslaunch | xargs ps --sort=cmd --ppid | awk   '{OFS=\",\";ORS=\" \"} $1!~/PID/ {print \"-p \"$1}' | xargs top -o -COMMAND -b -n1 | awk '{ORS=\"\"} $1 ~ /^[1-9].*/ {$1=$1; print $9,$10\",\"} END{print \"\\n\"}' OFS=\",\" >> " + *file;
        //std::cout << s ;
        system(s.c_str());
        ros::Duration dur = ros::Time::now() - *t_begin;
        time_stamps.push_back(dur.toSec());
        ros::Duration dur_test = ros::Time::now() - test;
        double tmp_t = dur_test.toSec()*1000;
        //std::cout << "duration of monitor resources: " << tmp_t/1000 << "s\n";
        if (tmp_t < 500){
            std::this_thread::sleep_for(std::chrono::milliseconds((int)rint(500-tmp_t)));
        }
    }
}

monitor_results* end_monitor(std::unique_ptr<std::thread>& monitor_ptr,std::string* file, ros::Time* begin,sqlite3* db, int iteration, int goal_nr){
    stop_monitor = true;
    monitor_ptr->join();
    ros::Duration dur = ros::Time::now() - *begin;
    stop_monitor = false; //enables monitoring in next iteration
    std::ifstream f;
    f.open(*file);
    std::string word;
    std::string line;
    int i = 0;
    double cpu_avg = 0;
    double mem_avg = 0;
    if (!f.is_open()) {
        std::cout << "file closed";
        return 0;}
    while(std::getline(f,line) && !line.empty()){
        std::istringstream iss(line);
        int c  = 0;
        std::vector<double> res;
        while(std::getline(iss,word, ',')){
            //std::cout << word << " ";
            res.push_back(atof(word.c_str()));
            if(c%2 == 0) {
                cpu_avg += res[c];}
            else {mem_avg += res[c];}
            c++;
        }
        if (c < 16 || res.size() < 16){
            std::cout << "error, not correct amount of values to monitor! " << c << " " << res.size() << "\n";
        }
        else{
            std::string sql = "INSERT INTO res_monitor VALUES (" + std::to_string(iteration) + "," + std::to_string(goal_nr) +
            ","+ std::to_string(time_stamps[i]) + "," + std::to_string(res[0]) + "," + std::to_string(res[1]) +
            ","+ std::to_string(res[2]) + ","+ std::to_string(res[3]) + "," + std::to_string(res[4]) +
            ","+ std::to_string(res[5]) + ","+ std::to_string(res[6]) + ","+ std::to_string(res[7]) +
            "," + std::to_string(res[8]) + ","+ std::to_string(res[9]) + ","+ std::to_string(res[10]) +
            "," + std::to_string(res[11]) + "," + std::to_string(res[12] + res[14]) + "," + std::to_string(res[13] + res[15]) + ");";
            int j = insertSQLite(db,&sql);
            }
        i++;
        //std::cout << i;
    }
    if (i != time_stamps.size()){
        std::cout << "error, incorrect number of time_stamps "<< time_stamps.size() << " nr_insertions: "<< i <<"\n";
    }
    std::cout << "inserted " << i << " entries resource monitoring into database \n";
    f.close();
    monitor_results* ret = (monitor_results*) malloc(sizeof(monitor_results));
    //res->pid = 0;
    if (cpu_avg != cpu_avg || mem_avg != mem_avg || !i){
        ret->cpu_avg = 0;
        ret->mem_avg = 0;
    }else{
        ret->cpu_avg = cpu_avg / i;
        ret->mem_avg = mem_avg / i;
    }
    ret->duration = dur.toSec(); //dur.sec +dur.nsec*10e-9;
    ret->clock_count = i;
    time_stamps.clear();
    return ret;
}

