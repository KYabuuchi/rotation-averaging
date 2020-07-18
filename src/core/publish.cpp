#define _USE_MATH_DEFINES
#include "core/publish.hpp"
#include <cmath>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>


namespace pub
{
void publishText(ros::Publisher& publisher, const std::string& text)
{
  geometry_msgs::Vector3 diameter;
  diameter.z = 0.2;

  int id = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "text";
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 1.5;
  marker.pose.position.y = 1.5;
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
  marker.text = text;

  publisher.publish(marker);
}

Visualizer::Visualizer(int V, ros::Publisher& publisher) : V(V), visualizatin_marker_array_publisher(publisher)
{
  for (int i = 0; i < V; i++) {
    double tmp = 2 * M_PI / static_cast<double>(V);
    Eigen::Vector3d t(std::cos(tmp * static_cast<double>(i)), std::sin(tmp * static_cast<double>(i)), 0);
    tvecs.push_back(t);
  }

  // primary
  color[0].r = color[1].g = color[2].b = 1.0;
  // alpha
  color[0].a = color[1].a = color[2].a = 0.2;
  // other
  color[0].g = color[0].b = color[1].r = color[1].b = color[2].r = color[2].g = 0.2;
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

  int id = 0;
  visualization_msgs::MarkerArray marker_array;
  for (size_t i = 0; i < V; i++) {
    for (size_t j = 0; j < V; j++) {
      if (measurements.count(std::make_pair(i, j)) == 0) continue;
      Eigen::Matrix3d R = measurements.at(std::make_pair(i, j));
      marker_array.markers.push_back(makeMarker(id++, estimated.at(i) * R, tvecs.at(j)));
    }
  }
  visualizatin_marker_array_publisher.publish(marker_array);
}

visualization_msgs::Marker Visualizer::makeMarker(int id, const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "cone";
  marker.id = id;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation = rotationMatrix2GeometryQuaternion(R);
  marker.pose.position.x = t.x();
  marker.pose.position.y = t.y();
  marker.pose.position.z = t.z();
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;


  const double Length = 0.3;   // length
  const double Radius = 0.05;  // radius
  const int N = 8;             // resolution

  for (int axis = 0; axis < 3; axis++) {

    Eigen::Vector3d s(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    t(axis) = Length;
    Eigen::Vector3d n(0, 0, 0);
    n((axis + 1) % 3) = Radius;
    Eigen::Vector3d m(0, 0, 0);
    m((axis + 2) % 3) = Radius;

    Eigen::Vector3d nm = std::cos(2 * (N - 1) * M_PI / N) * n + std::sin(2 * (N - 1) * M_PI / N) * m;
    for (int i = 0; i < N; i++) {
      marker.points.push_back(vector3d2GeometryMsg(s));
      marker.points.push_back(vector3d2GeometryMsg(t + nm));
      nm = std::cos(2 * i * M_PI / N) * n + std::sin(2 * i * M_PI / N) * m;
      marker.points.push_back(vector3d2GeometryMsg(t + nm));

      marker.colors.push_back(color[axis]);
      marker.colors.push_back(color[axis]);
      marker.colors.push_back(color[axis]);
    }
  }

  return marker;
}

}  // namespace pub