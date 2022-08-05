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
using namespace std;

std::atomic<bool> stop_monitor(false);

void monitor_process(pid_t* pid, string* file) {
    std::cout << "pid: " << to_string(*pid) << "\n";
    //ros::Duration sleep_rate(0.5);
    int pid1 = 3524;
    while (!stop_monitor.load()) {
        system(("top -b -n1 -p " + to_string(pid1) + " | awk '$1 ~  /^" + to_string(pid1) +
                "/ {print $1,$9,$10,$11,$12}' >> " + *file).c_str());
        //sleep_rate.sleep();
        this_thread::sleep_for(chrono::milliseconds(500));
        //cout << "Not stopping yet\n" ;
    }
}

void end_monitor(std::thread* monitor){
    stop_monitor = true;
    monitor->join();
    cout << "finished monitoring " << endl;
    //stop_monitor = true;
}
int main(int argc,char **argv) {
    pid_t pid = getpid();
    string file = "/home/kotname/test2.txt";
    std::thread monitor(monitor_process,&pid,&file);
    time_t timer = time(NULL);
    this_thread::sleep_for(chrono::seconds(2));
    end_monitor(&monitor);
    double dur = difftime(time(NULL),timer);
    ifstream f;
    f.open(file);
    string word;
    string line;
    double cpu_avg = 0;
    double mem_avg = 0;
    int i = 0;
    cout << "start processing file \n";
    while(getline(f,line)){
        istringstream iss(line) ;
        int c  = 0;
        while(getline(iss,word, ' ')){
            cout << word << " ";
            if (c == 1) {
                try {
                    cpu_avg += atof(word.c_str());
                }catch (int n){
                    cout << "Fehler cpu";
                }
            }else if (c==2){
                try{
                    mem_avg += atof(word.c_str());
                }catch (int n){
                    cout << "Fehler mem";
                }
            }
        c++;
        }
    i++;
    cout << endl;
    }
    cout << "cpu_avg: " << cpu_avg/i << " mem_avg: " << mem_avg/i << " duration: " << dur <<" sec nr_values: " << i << endl;
    f.close();
}
