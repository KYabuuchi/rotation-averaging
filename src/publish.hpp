#pragma once
#include <ros/ros.h>

namespace pub
{
void publishPose();

void publishMeasurement(ros::Publisher& publisher);

void publishIterator(ros::Publisher& publisher, int iteration);
}  // namespace pub