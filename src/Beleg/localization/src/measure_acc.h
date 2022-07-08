//
// Created by kotname on 01.07.22.
//

#ifndef ROS_WS_MOVE_ON_PATH_H
#define ROS_WS_MOVE_ON_PATH_H

void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom_msg);
void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg);
int main(int argc,char **argv);

#endif //ROS_WS_MOVE_ON_PATH_H
