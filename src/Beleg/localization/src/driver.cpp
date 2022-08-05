#include "driver.h"
// drive the given distance with given constant speed, then stop
void drive(ros::Publisher& velocity_pub, const double distance, const double speed, const double proc_x_mean) {
   const int REFRESHRATE = 20;
   ros::Rate loop_rate(REFRESHRATE);

   int counter = 0;
   double duration = distance / abs(speed);

   while (ros::ok()) {
      ros::spinOnce();

      ++counter;
      geometry_msgs::Twist velocity;

      velocity.linear.x = speed * (1 - proc_x_mean);
      velocity.linear.y = 0;
      velocity.linear.z = 0;

      velocity.angular.x = 0;
      velocity.angular.y = 0;
      velocity.angular.z = 0;
      //first and last message with half speed
      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(REFRESHRATE) + 1/REFRESHRATE >= duration) {
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


void rotate(ros::Publisher& velocity_pub, double yaw, const double ang_vel,const double proc_yaw_mean) {
   const int REFRESHRATE = 20;
   ros::Rate loop_rate(REFRESHRATE);
   yaw = yaw * (1 - proc_yaw_mean);
   int counter = -1;
    //only positive yaw values ingoing -> rotation clockwise if yaw too big
   if (yaw > 2*M_PI){
       yaw = fmod(yaw,2*M_PI);
   }else {
       if (yaw > M_PI) {
           yaw = yaw - 2*M_PI;
       }else if(yaw < 0){
           if (yaw < 2*-M_PI){
               std::cout << "negative yaw received in rotation: " << yaw << "\n";
               yaw = fmod(yaw,2*M_PI);
           }else if (yaw < -M_PI){
               std::cout << "negative yaw received in rotation: " << yaw << "\n";
               yaw = 2*M_PI- abs(yaw);
           }
       }
   }
   auto current_yaw = yaw >= 0 ? ang_vel : -ang_vel;
   double duration = yaw /current_yaw;

   //cout << "dauer der Drehung: " << duration << " drehung insgesamt um: " << yaw << "\n";
   while (ros::ok()) {
      ros::spinOnce();

      ++counter;

      geometry_msgs::Twist velocity;

      velocity.linear.x = 0;
      velocity.linear.y = 0;
      velocity.linear.z = 0;

      velocity.angular.x = 0;
      velocity.angular.y = 0;
      velocity.angular.z = current_yaw;
      if (counter == 0 || (counter + 1)  / static_cast<double>(REFRESHRATE) + 1/REFRESHRATE >= duration){
          velocity.angular.z = current_yaw/2;
      }
      velocity_pub.publish(velocity);

      // check if duration has passed
      if (counter / static_cast<double>(REFRESHRATE) + 1/REFRESHRATE >= duration) {
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