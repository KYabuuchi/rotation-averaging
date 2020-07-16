#define _USE_MATH_DEFINES
#include "publish.hpp"
#include <cmath>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>


namespace pub
{
void publishError(ros::Publisher& publisher, double error)
{
  std_msgs::Float32 error_msg;
  error_msg.data = static_cast<float>(error);
  publisher.publish(error_msg);
}


void publishIterator(ros::Publisher& publisher, int iteration)
{
  std_msgs::Float32 iteration_msg;
  iteration_msg.data = static_cast<float>(iteration);
  publisher.publish(iteration_msg);
}

void publishTime(ros::Publisher& publisher, int time_ms)
{
  std_msgs::Float32 time_msg;
  time_msg.data = static_cast<float>(time_ms);
  publisher.publish(time_msg);
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

Visualizer::Visualizer(int V, ros::Publisher& publisher) : V(V), visualizatin_marker_array_publisher(publisher)
{
  for (int i = 0; i < V; i++) {
    double tmp = 2 * M_PI / static_cast<double>(V);
    Eigen::Vector3d t(std::cos(tmp * static_cast<double>(i)), std::sin(tmp * static_cast<double>(i)), 0);
    tvecs.push_back(t);
  }
}

void Visualizer::publish(const Matrix3dVector& estimated, const RelativeRotations& measurements)
{
  for (int i = 0; i < V; i++) {
    Eigen::Matrix4d T;
    T.setIdentity();
    T.topRightCorner(3, 1) = tvecs.at(i);
    T.topLeftCorner(3, 3) = estimated.at(i);

    tf::Transform transform;
    transform.setFromOpenGLMatrix(T.cast<double>().eval().data());
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pose" + std::to_string(i)));
  }


  for (size_t i = 0; i < V - 1; i++) {
    for (size_t j = i + 1; j < V; j++) {
      if (!measurements.count(std::make_pair(i, j)) == 0) continue;
      Eigen::Matrix3d R = measurements.at(std::make_pair(i, j));
      // TODO: draw cones
    }
  }
}


}  // namespace pub