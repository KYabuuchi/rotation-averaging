#define _USE_MATH_DEFINES
#include "publish.hpp"
#include <cmath>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>


namespace pub
{
void publishIterator(ros::Publisher& publisher, int iteration)
{
  std_msgs::Float32 iteration_msg;
  iteration_msg.data = static_cast<float>(iteration);
  publisher.publish(iteration_msg);
}

void publishPose()
{
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(1, 0, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pose1"));
}

void publishMeasurement(ros::Publisher& publisher)
{
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::Vector3 diameter;
  diameter.z = 1.0;

  int id = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "text";
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0.5;
  marker.pose.position.y = 0.5;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  marker.id = static_cast<int>(id);
  marker.scale = diameter;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.text = "Is this a pen?\nNo, it's Covid-19.";
  marker_array.markers.push_back(marker);

  publisher.publish(marker_array);
}

Visualizer::Visualizer(int V) : V(V)
{
  for (int i = 0; i < V; i++) {
    double tmp = 2 * M_PI / static_cast<double>(V);
    Eigen::Vector3d t(std::cos(tmp * static_cast<double>(i)), std::sin(tmp * static_cast<double>(i)), 0);
    tvecs.push_back(t);
  }
}

void Visualizer::setPublisher(ros::Publisher& _publisher)
{
  publisher = _publisher;
}

void Visualizer::publish(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d>>& pairs)
{
  for (int i = 0; i < V; i++) {
    Eigen::Matrix4d T;
    T.setIdentity();
    T.topRightCorner(3, 1) = tvecs.at(i);

    tf::Transform transform;
    transform.setFromOpenGLMatrix(T.cast<double>().eval().data());
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pose" + std::to_string(i)));
  }
}


}  // namespace pub