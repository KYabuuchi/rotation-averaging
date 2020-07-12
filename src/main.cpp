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
  ros::Publisher iteration_publisher = nh.advertise<std_msgs::Float32>("ra/iteration", 1);
  ros::Publisher pose_publisher = nh.advertise<visualization_msgs::MarkerArray>("ra/pose", 1);
  ros::Publisher time_publisher = nh.advertise<std_msgs::Float32>("ra/time", 1);
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
    // Print the current state
    for (int i = 0; i < problem.V; i++) {
      Eigen::Matrix3d opt_R = Y.block(0, 3 * i, 3, 3);
      double theta = util::calcAngleResidual(problem.truth(i), util::normalize(opt_R));
      std::cout << i << " cordal distance= " << theta << std::endl;
    }
    std::cout << "\033[1;32m###################\033[0m" << std::endl;


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

    std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d>> a;
    visualizer.publish(a);

    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}