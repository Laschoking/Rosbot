#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "helpers.h"

#include <iostream>
#include <string>
#include <sstream>

const int refreshrate = 50;

ros::Publisher velocity_pub;

// drive the given distance with given constant speed, then stop
void drive(const double distance, const double speed) {
   ros::Rate loop_rate(refreshrate);

   int counter = 0;

   double duration = distance / speed;

   auto current_speed = speed;

   std::cout << "current speed: " << current_speed << std::endl;

   while (ros::ok()) {
      ros::spinOnce();

      ++counter;

      geometry_msgs::Twist velocity;

      velocity.linear.x = current_speed;
      velocity.linear.y = 0;
      velocity.linear.z = 0;

      velocity.angular.x = 0;
      velocity.angular.y = 0;
      velocity.angular.z = 0;

      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(refreshrate) >= duration) {
         break;
      }

      loop_rate.sleep();
   }

   geometry_msgs::Twist velocity;

   velocity.linear.x = 0;
   velocity.linear.y = 0;
   velocity.linear.z = 0;

   velocity.angular.x = 0;
   velocity.angular.y = 0;
   velocity.angular.z = 0;

   // stop motors
   velocity_pub.publish(velocity);

}

int main(int argc, char **argv) {

   double distance = 0;
   double speed = 0;

   ros::init(argc, argv, "process_error");
   ros::NodeHandle n("~");

   // distance in some unit
   n.param<double>("distance", distance, 1);

   // speed in distance units per second
   n.param<double>("speed", speed, 1);

   std::cout << "distance: " << distance << std::endl;
   std::cout << "speed: " << speed << std::endl;
   
   velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

   bool cont = true;
   do {
      std::ostringstream os;

      distance = getDoubleInput("Distance to drive", distance);
      speed = getDoubleInput("Speed to drive with", speed);

      std::cout << "Driving with velocity " << speed << " and distance " << distance << "..." << std::endl;
      
      drive(distance, speed);

      cont = getBoolInput("Continue driving");
   } while(cont);

}
