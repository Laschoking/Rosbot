#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "helpers.h"

#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <chrono>
#include <algorithm>

/*
Small program to make it easier to measure process errors
Will drive with a given speed and distance, then waits for the measurement to be done
Then pressing enter will make it move 90% back to its original position (still needs to be manually put in the correct place again)
Then you can press enter to repeat the measurement with the same distance and speed, or enter new distance and speed for other measurements
*/

int main(int argc, char **argv) {

   double distance = 0;
   double speed = 0;

   ros::init(argc, argv, "process_error");
   ros::NodeHandle n("~");

   // get distance parameter
   n.param<double>("distance", distance, 1);
	
   // get speed parameter
   n.param<double>("speed", speed, 1);

   // wait for other packages to be ready before printing
   std::this_thread::sleep_for(std::chrono::seconds(5));

   std::cout << "distance: " << distance << std::endl;
   std::cout << "speed: " << speed << std::endl;
   
   ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

   bool cont = true;
   do {
      std::ostringstream os;

      if (getBoolInput("Change distance or speed", false)) {
         distance = getDoubleInput("Distance to drive", distance);
         speed = getDoubleInput("Speed to drive with", speed);
      }

      std::cout << "Driving with velocity " << speed << " and distance " << distance << "..." << std::endl;
      
      drive(velocity_pub, distance, speed);

      if (getBoolInput("Drive back", true)) {
         // go back slowly
         drive(velocity_pub, -std::max(distance - 0.12, 0.), -0.5);
         std::cout << "Drove back" << std::endl;
      }
   } while(cont);

   exit(0);
}
