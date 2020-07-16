#include "cone_publisher.hpp"
#include "problem_generator.hpp"
#include "publish.hpp"
#include "rotation_averaging.hpp"
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
  int vertex_num = 10;
  double noise_gain = 0.1;
  pnh.getParam("vertex_num", vertex_num);
  pnh.getParam("noise_gain", noise_gain);
  ROS_INFO("# of Vertex is %d.", vertex_num);


  // Declear publisher
  ros::NodeHandle nh;
  ros::Publisher iteration_publisher = nh.advertise<std_msgs::Float32>("/iteration", 1);
  ros::Publisher time_publisher = nh.advertise<std_msgs::Float32>("/time", 1);
  ros::Publisher error_publisher = nh.advertise<std_msgs::Float32>("/error", 1);
  ros::Publisher measurement_publisher = nh.advertise<visualization_msgs::MarkerArray>("/measurements", 1);
  pub::Visualizer visualizer(vertex_num, measurement_publisher);


  // Construct the problem
  ProblemGenerator problem(vertex_num, noise_gain);
  RotationAveraging ra(vertex_num);
  for (size_t i = 0; i < vertex_num - 1; i++) {
    for (size_t j = i + 1; j < vertex_num; j++) {
      if (!problem.isAdjacent(i, j)) continue;
      ra.setMeasurement(i, j, problem.measured(i, j));
    }
  }


  // Setup main loop
  ros::Rate loop_rate(1);
  int iteration = 0;
  int past_time = 0;


  while (ros::ok()) {
    // Print the current state
    std::cout << "\033[1;32m###################" << iteration << "\033[0m" << std::endl;
    double error = ra.getTotalError();
    std::cout << "error: " << error << ", time: " << past_time << std::endl;


    // Publish texts for RViz
    pub::publishIterator(iteration_publisher, iteration);
    pub::publishTime(time_publisher, past_time);
    pub::publishError(error_publisher, error);


    // Publish pose for RViz
    Matrix3dVector estimated;
    for (size_t i = 0; i < vertex_num; i++) {
      Eigen::Matrix3d R;
      ra.getAbsolute(i, R);
      estimated.push_back(R);
    }
    visualizer.publish(estimated, problem.getMeasurement());


    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();
    iteration++;


    // Optimize
    auto start = std::chrono::system_clock::now();
    if (iteration % 2 == 0)
      ra.optimize();

    auto end = std::chrono::system_clock::now();
    past_time += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  }

  return 0;
}