#include "cone_publisher.hpp"

ConePublisher::ConePublisher(ros::Publisher& publisher) : publisher(publisher)
{
  // primary
  color[0].r = color[1].g = color[2].b = 1.0;
  // alpha
  color[0].a = color[1].a = color[2].a = 0.2;
  // other
  color[0].g = color[0].b = color[1].r = color[1].b = color[2].r = color[2].g = 0.2;
}

void ConePublisher::publish()
{
  const double L = 0.3;  // length
  const double R = 0.1;  // radius
  const int N = 8;

  visualization_msgs::Marker marker = initializeMarker();

  for (int axis = 0; axis < 3; axis++) {

    Eigen::Vector3d s(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    t(axis) = L;
    Eigen::Vector3d n(0, 0, 0);
    n((axis + 1) % 3) = R;
    Eigen::Vector3d m(0, 0, 0);
    m((axis + 2) % 3) = R;

    Eigen::Vector3d nm = std::cos(2 * (N - 1) * M_PI / N) * n + std::sin(2 * (N - 1) * M_PI / N) * m;
    for (int i = 0; i < N; i++) {
      marker.points.push_back(convert(s));
      marker.points.push_back(convert(t + nm));
      nm = std::cos(2 * i * M_PI / N) * n + std::sin(2 * i * M_PI / N) * m;
      marker.points.push_back(convert(t + nm));

      marker.colors.push_back(color[axis]);
      marker.colors.push_back(color[axis]);
      marker.colors.push_back(color[axis]);
    }
  }
  publisher.publish(marker);
}

visualization_msgs::Marker ConePublisher::initializeMarker(int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "cone";
  marker.id = id;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  return marker;
}

geometry_msgs::Point ConePublisher::convert(const Eigen::Vector3d& v)
{
  geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
};
