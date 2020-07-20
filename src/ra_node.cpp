#include "core/problem_generator.hpp"
#include "core/publish.hpp"
#include "duality/rotation_averaging.hpp"
#include <chrono>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // Set random seed
  unsigned int seed = 0;
  srand(seed);


  // rosparams
  ros::init(argc, argv, "ra_node");
  ros::NodeHandle pnh("~");
  int vertex_num = 10;
  double noise_gain = 1.0;
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


  // setup initial absolute rotation using integrated measurement
  Matrix3dVector initial;
  initial.push_back(Eigen::Matrix3d::Identity());
  for (int i = 0; i < vertex_num - 1; i++) {
    initial.push_back(initial.at(i) * problem.measured(i, i + 1));
  }
  ra.setAbsolute(initial);


  // Setup main loop
  ros::Rate loop_rate(2);
  int iteration = 0;
  int past_time = 0;


  int wait_for_optimization = 0;
  while (ros::ok()) {
    // Print the current state
    double error = ra.getTotalSquaredError();
    std::cout << "\033[1;32m################### " << iteration << " " << error << " " << past_time << "\033[0m" << std::endl;


    // Publish information for RViz
    std::string text = "time: " + std::to_string(past_time / 1000.0) + "[ms]\nerror: " + std::to_string(error) + "\niteration: " + std::to_string(iteration);
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


    // Optimize
    wait_for_optimization++;
    if (wait_for_optimization >= 10) {
      auto start = std::chrono::system_clock::now();
      ra.optimize();
      // ra.optimizeOnce();
      auto end = std::chrono::system_clock::now();
      iteration++;
      past_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
  }

  return 0;
}
