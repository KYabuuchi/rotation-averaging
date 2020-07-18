#include "core/problem_generator.hpp"
#include "core/publish.hpp"
#include "g2o/g2o_types.hpp"
#include <chrono>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <iostream>
#include <ros/ros.h>

// // setup the solver
// typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> MyBlockSolver;
// typedef g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType> MyLinearSolver;
// g2o::SparseOptimizer optimizer;
// optimizer.setVerbose(false);
// g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
//     g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
// optimizer.setAlgorithm(solver);

// // build the optimization problem given the points
// // 1. add the circle vertex
// VertexRotation* circle = new VertexCircle();
// circle->setId(0);
// circle->setEstimate(Eigen::Vector3d(3.0, 3.0, 3.0));  // some initial value for the circle
// optimizer.addVertex(circle);

// for (int i = 0; i < numPoints; ++i) {
//   EdgePointOnCircle* e = new EdgePointOnCircle;
//   e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
//   e->setVertex(0, circle);
//   e->setMeasurement(points[i]);
//   optimizer.addEdge(e);
// }

// // perform the optimization
// optimizer.initializeOptimization();
// optimizer.setVerbose(verbose);
// optimizer.optimize(maxIterations);


int main(int argc, char** argv)
{
  // Set random seed
  unsigned int seed = 0;
  srand(seed);


  // rosparams
  ros::init(argc, argv, "ra_g2o_node");
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


  // setup initial absolute rotation using integrated measurement
  Matrix3dVector initial;
  initial.push_back(Eigen::Matrix3d::Identity());
  for (int i = 0; i < vertex_num - 1; i++) {
    initial.push_back(initial.at(i) * problem.measured(i, i + 1));
  }
  // ra.setAbsolute(initial);


  // Setup main loop
  ros::Rate loop_rate(1);
  int iteration = 0;
  int past_time = 0;


  int wait_for_optimization = 0;
  while (ros::ok()) {
    // Print the current state
    // double error = ra.getTotalError();
    double error = 0;
    std::cout << "\033[1;32m################### " << iteration << "\033[0m" << std::endl;


    // Publish information for RViz
    std::string text = "time: " + std::to_string(past_time / 1000.0) + "[ms]\nerror: " + std::to_string(error) + "\niteration: " + std::to_string(iteration);
    pub::publishText(text_publisher, text);


    // Publish pose for RViz
    Matrix3dVector estimated;
    for (size_t i = 0; i < vertex_num; i++) {
      Eigen::Matrix3d R;
      // ra.getAbsolute(i, R);
      R.setIdentity();
      estimated.push_back(R);
    }
    visualizer.publish(estimated, problem.getMeasurement());


    // Spin and wait
    ros::spinOnce();
    loop_rate.sleep();


    // Optimize
    wait_for_optimization++;
    if (wait_for_optimization >= 3) {
      auto start = std::chrono::system_clock::now();
      // ra.optimize();
      // ra.optimizeOnce();
      auto end = std::chrono::system_clock::now();
      iteration++;
      past_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
  }

  return 0;
}
