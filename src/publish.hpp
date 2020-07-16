#pragma once
#include "types.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace pub
{
void publishMeasurement(ros::Publisher& publisher);

void publishIterator(ros::Publisher& publisher, int iteration);

void publishError(ros::Publisher& publisher, double error);

void publishTime(ros::Publisher& publisher, int time_ms);


class Visualizer
{
public:
  Visualizer(int V, ros::Publisher& publisher);

  void publish(const Matrix3dVector& estimated, const RelativeRotations& measurements);


private:
  ros::Publisher visualizatin_marker_array_publisher;

  const int V;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs;
  tf::TransformBroadcaster br;
};
}  // namespace pub