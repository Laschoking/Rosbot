//
// Created by kotname on 04.08.22.
//
#include <iostream>
#include "monitor_process.h"
#include "sys/types.h"
#include "sys/sysinfo.h"
#include <unistd.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <atomic>
#include <thread>
#include <time.h>
#include <sstream>
#include <fstream>

struct sysinfo memInfo;

std::atomic<bool> stop_monitor(false);

void monitor_process(pid_t* pid, std::string* file) {
    std::cout << "pid: " << std::to_string(*pid) << "\n";
    //ros::Duration sleep_rate(0.5);
    while (!stop_monitor.load()) {
        system(("top -cb -n1 -p " + std::to_string(*pid) + " | awk '$1 ~  /^" + std::to_string(*pid) +
                "/ {print $1,$9,$10,$11,$12}' >> " + *file).c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

monitor_results* end_monitor(std::unique_ptr<std::thread>& monitor_ptr,std::string* file, ros::Time* begin){
    stop_monitor = true;
    monitor_ptr->join();
    ros::Duration dur = *begin - ros::Time::now();
    std::cout << "finished monitoring " << std::endl;
    stop_monitor = true;
    std::ifstream f;
    f.open(*file,std::ios_base::trunc);
    std::string word;
    std::string line;
    double cpu_avg = 0;
    double mem_avg = 0;
    int i = 0;
    std::cout << "start processing file \n";
    while(getline(f,line)){
        std::istringstream iss(line) ;
        int c  = 0;
        while(getline(iss,word, ' ')){
            std::cout << word << " ";
            if (c == 1) {
                try {
                    cpu_avg += atof(word.c_str());
                }catch (int n){
                    std::cout << "Fehler cpu";
                }
            }else if (c==2){
                try{
                    mem_avg += atof(word.c_str());
                }catch (int n){
                    std::cout << "Fehler mem";
                }
            }
        c++;
        }
    i++;
    //std::cout << std::endl;
    }
    std::cout << "finished monitoring,count of cycles: " << i << "\n";
    f.close();
    monitor_results* res = (monitor_results*) malloc(sizeof(monitor_results));
    //res->pid = 0;
    res->cpu_avg = cpu_avg;
    res->mem_avg = mem_avg;
    res->duration = dur.sec +dur.nsec*10e-9;
    res->clock_count = i;
    return res;
}

