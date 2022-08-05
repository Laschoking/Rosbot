//
// Created by kotname on 04.08.22.
//

#ifndef ROS_WS_MONITOR_PROCESS_H
#define ROS_WS_MONITOR_PROCESS_H
#include <ros/ros.h>
#include <thread>
void monitor_process(pid_t* pid, std::string* file);

struct monitor_results{
        int pid;
        double cpu_avg;
        double mem_avg;
        double duration;
        int clock_count;};


monitor_results* end_monitor(std::unique_ptr<std::thread>& monitor_ptr,std::string* file, ros::Time* begin);

#endif //ROS_WS_MONITOR_PROCESS_H
