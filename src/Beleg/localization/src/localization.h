#pragma once

#include <ros/ros.h>
#include <string>

void hectorCallback(boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> hectorPose);

