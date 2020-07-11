#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

void publishMeasurement(ros::Publisher& publisher)
{
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::Vector3 diameter;
  diameter.x = 4.0;
  diameter.y = 4.0;
  diameter.z = 1.0;

  int id = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "poses";
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  marker.id = static_cast<int>(id);
  marker.scale = diameter;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.3f;
  marker_array.markers.push_back(marker);

  publisher.publish(marker_array);
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ra_node");

  ros::NodeHandle nh;
  ros::Publisher iteration_publisher = nh.advertise<std_msgs::Float32>("ra/iteration", 1);
  ros::Publisher pose_publisher = nh.advertise<visualization_msgs::MarkerArray>("ra/pose", 1);

  std::chrono::system_clock::time_point m_start;
  ros::Rate loop_rate(5);
  int iteration = 0;

  while (ros::ok()) {
    {
      std_msgs::Float32 iteration_msg;
      iteration_msg.data = static_cast<float>(iteration);
      iteration_publisher.publish(iteration_msg);
    }
    publishMeasurement(pose_publisher);
    publishPose();

    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();
  }
}