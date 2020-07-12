#pragma once
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace pub
{
void publishPose();

void publishMeasurement(ros::Publisher& publisher);

void publishIterator(ros::Publisher& publisher, int iteration);

class Visualizer
{
public:
  Visualizer(int V);

  void setPublisher(ros::Publisher& publisher);

  void publish(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d>>& pairs);

private:
  ros::Publisher publisher;
  const int V;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs;
  tf::TransformBroadcaster br;
};
}  // namespace pub