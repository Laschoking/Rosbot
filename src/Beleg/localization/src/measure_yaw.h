//pre
// Created by kotname on 01.07.22.
//
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "driver.h"
#ifndef ROS_WS_MEASURE_YAW_H
#define ROS_WS_MEASURE_YAW_H

void odomCallback(boost::shared_ptr<const nav_msgs::Odometry> odom_msg);

void imuCallback(boost::shared_ptr<const sensor_msgs::Imu> imu_msg);

void manage_states(double yaw, bool source);

void integrateImu(boost::shared_ptr<const sensor_msgs::Imu> imu_msg);
//void rotate_vehicle(double angle, double distance);

#endif
