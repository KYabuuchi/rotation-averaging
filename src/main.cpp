#include "cone_publisher.hpp"
#include "problem_generator.hpp"
#include "publish.hpp"
#include "ra.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>


int main(int argc, char** argv)
{
  // Initialize random seed
  unsigned int seed = 0;
  srand(seed);


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
  ConePublisher cone(cone_publisher);


  // Construct the problem
  ProblemGenerator problem(vertex_num, noise_gain);
  RotationAveraging ra(vertex_num);

  // どうせサイクリックなので，
  for (size_t i = 0; i < vertex_num - 1; i++) {
    for (size_t j = i + 1; j < vertex_num; j++) {
      ra.setMeasurement(i, j, problem.measured(i, j));
    }
  }

  // Setup main loop
  std::chrono::system_clock::time_point m_start;
  ros::Rate loop_rate(1);
  int iteration = 0;


  while (ros::ok()) {
    // Print the current state
    std::cout << "\033[1;32m###################" << iteration << "\033[0m" << std::endl;
    for (size_t i = 0; i < vertex_num - 1; i++) {
      for (size_t j = i + 1; j < vertex_num; j++) {
        std::cout << i << " " << j << " " << ra.getError(i, j) << std::endl;
      }
    }

    // Optimize
    ra.optimize();

    // // Publish for RViz
    // pub::publishMeasurement(pose_publisher);
    // pub::publishPose();
    // pub::publishIterator(iteration_publisher, iteration);
    // cone.publish();

    // std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d>> a;
    // visualizer.publish(a);

    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();
    iteration++;
  }

  return 0;
}