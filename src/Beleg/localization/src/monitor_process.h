//
// Created by kotname on 04.08.22.
//

#ifndef ROS_WS_MONITOR_PROCESS_H
#define ROS_WS_MONITOR_PROCESS_H
#include <ros/ros.h>
#include <thread>
#include <sqlite3.h>
struct monitor_results{
        double cpu_avg;
        double mem_avg;
        double duration;
        int clock_count;};
void monitor_process(std::string* file,ros::Time* t_begin);
monitor_results* end_monitor(std::unique_ptr<std::thread>& monitor_ptr,std::string* file, ros::Time* begin,sqlite3* db, int iteration, int goal_nr);



#endif //ROS_WS_MONITOR_PROCESS_H
