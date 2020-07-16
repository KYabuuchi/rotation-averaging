#pragma once
#include "types.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

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

  geometry_msgs::Quaternion rotationMatrix2GeometryQuaternion(const Eigen::Matrix3d& R)
  {
    Eigen::Quaterniond eigen_q(R);
    geometry_msgs::Quaternion ros_q;
    ros_q.w = eigen_q.w();
    ros_q.x = eigen_q.x();
    ros_q.y = eigen_q.y();
    ros_q.z = eigen_q.z();
    return ros_q;
  }

  geometry_msgs::Point vector3d2GeometryMsg(const Eigen::Vector3d& v)
  {
    geometry_msgs::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
  };

  visualization_msgs::Marker makeMarker(int id, const Eigen::Matrix3d& R, const Eigen::Vector3d& t);


  const int V;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs;
  tf::TransformBroadcaster br;
  std_msgs::ColorRGBA color[3];
};
}  // namespace pub