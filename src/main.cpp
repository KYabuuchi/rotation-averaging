#include "problem.hpp"
#include "publish.hpp"
#include "ra.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

void publishCone(ros::Publisher& publisher)
{
  Eigen::Matrix4d T;
  T.Identity();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "cone";
  marker.id = 0;
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

  const int N = 8;
  std_msgs::ColorRGBA color[3];
  color[0].r = 1.0;
  color[0].g = 0.2;
  color[0].b = 0.2;
  color[0].a = 0.2;
  color[1].r = 0.2;
  color[1].g = 1.0;
  color[1].b = 0.2;
  color[1].a = 0.2;
  color[2].r = 0.2;
  color[2].g = 0.2;
  color[2].b = 1.0;
  color[2].a = 0.2;


  auto convert = [](const Eigen::Vector3d& v) -> geometry_msgs::Point {
    geometry_msgs::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
  };

  const double L = 0.3;  // length
  const double R = 0.1;  // radius

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

int main(int argc, char** argv)
{
  // rosparams
  ros::init(argc, argv, "ra_node");
  ros::NodeHandle pnh("~");
  int vertex_num = 3;
  double noise_gain = 0.1;
  pnh.getParam("vertex_num", vertex_num);
  pnh.getParam("noise_gain", noise_gain);
  ROS_INFO("# of Vertex is %d.", vertex_num);


  // Declear publisher
  ros::NodeHandle nh;
  ros::Publisher iteration_publisher = nh.advertise<std_msgs::Float32>("/iteration", 1);
  ros::Publisher pose_publisher = nh.advertise<visualization_msgs::MarkerArray>("/pose", 1);
  ros::Publisher cone_publisher = nh.advertise<visualization_msgs::Marker>("/cone", 1);
  ros::Publisher time_publisher = nh.advertise<std_msgs::Float32>("/time", 1);
  pub::Visualizer visualizer(vertex_num);


  // Initialize random seed
  unsigned int seed = 0;
  srand(seed);


  // Construct the problem
  Problem problem(vertex_num, noise_gain);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Identity(3 * vertex_num, 3 * vertex_num);  // 3V x 3V の適当な正定値行列
  Eigen::MatrixXd tilde_R = problem.getTildeR();


  // Setup main loop
  std::chrono::system_clock::time_point m_start;
  ros::Rate loop_rate(1);
  int iteration = 0;


  while (ros::ok()) {
    iteration++;
    // Print the current state
    for (int i = 0; i < problem.V; i++) {
      Eigen::Matrix3d opt_R = Y.block(0, 3 * i, 3, 3);
      double theta = util::calcAngleResidual(problem.truth(i), util::normalize(opt_R));
      std::cout << i << " cordal distance= " << theta << std::endl;
    }
    std::cout << "\033[1;32m###################" << iteration << "\033[0m" << std::endl;


    // Optimize
    for (int i = 0; i < problem.V; i++) {
      Eigen::MatrixXd Bk = ra::calcB(Y);        // 3(N-1) x 3(N-1)
      Eigen::MatrixXd Wk = ra::calcW(tilde_R);  // 3(N-1) x 3
      Eigen::MatrixXd Sk = ra::calcS(Bk, Wk);   // 3(N-1) x 3
      Y = ra::calcY(Sk, Bk);                    // 3N     x 3N
      ra::warp(tilde_R, Y);
    }


    // Publish for RViz
    pub::publishMeasurement(pose_publisher);
    pub::publishPose();
    pub::publishIterator(iteration_publisher, iteration);
    publishCone(cone_publisher);

    std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d>> a;
    visualizer.publish(a);

    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}