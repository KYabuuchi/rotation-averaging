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
  ros::Publisher text_publisher = nh.advertise<visualization_msgs::Marker>("/text", 1);
  ros::Publisher measurement_publisher = nh.advertise<visualization_msgs::MarkerArray>("/measurements", 1);
  pub::Visualizer visualizer(vertex_num, measurement_publisher);


  // Construct the problem
  ProblemGenerator problem(vertex_num, noise_gain);
  RotationAveraging ra(vertex_num);
  for (size_t i = 0; i < vertex_num; i++) {
    for (size_t j = 0; j < vertex_num; j++) {
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
    double error = ra.getTotalError();
    std::cout << "\033[1;32m###################" << iteration << "\033[0m" << std::endl;
    std::cout << "error: " << error << ", time: " << past_time << std::endl;


    // Publish information for RViz
    std::string text = "time: " + std::to_string(past_time) + "[ms]\nerror: " + std::to_string(error) + "\niteration: " + std::to_string(iteration / 2);
    pub::publishText(text_publisher, text);


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
    if (iteration % 2 == 0) {
      auto start = std::chrono::system_clock::now();
      ra.optimize();
      auto end = std::chrono::system_clock::now();
      past_time += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }
  }

  return 0;
}