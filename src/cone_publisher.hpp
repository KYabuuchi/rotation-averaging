#pragma once
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class ConePublisher
{
public:
  ConePublisher(ros::Publisher& publisher);
  void publish();

private:
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color[3];
  ros::Publisher publisher;


  visualization_msgs::Marker initializeMarker(int id = 0);

  geometry_msgs::Point convert(const Eigen::Vector3d& v);
};