#pragma once
#include "util.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

class Problem
{
public:
  Problem(const int V, const double noise_gain = 0.2) : V(V), noise_gain(noise_gain)
  {
    // Construct the adjacent graph
    adjacent_graph = util::initCyclicAdjacentGraph(V);
    std::cout << "adjacent graph=\n\033[1;31m"
              << adjacent_graph << "\033[0m" << std::endl;


    // Generate true rotation matrix
    true_rotations.push_back(Eigen::Matrix3d::Identity());
    for (int i = 1; i < V; i++) {
      true_rotations.push_back(util::randomRotation());
    }


    // Initialize random noise rotation matrix
    noise_rotation = Eigen::MatrixXd::Zero(3 * V, 3 * V);
    for (int i = 0; i < V; i++) {
      for (int j = 0; j < V; j++) {
        if (i == j) continue;
        if (i < j) continue;
        Eigen::Matrix3d N = util::noiseRotation(noise_gain);
        noise_rotation.block(3 * i, 3 * j, 3, 3) = N;
        noise_rotation.block(3 * j, 3 * i, 3, 3) = N.transpose();
      }
    }


    // Initialize measured rotation matrix
    measured_rotation = Eigen::MatrixXd::Zero(3 * V, 3 * V);
    for (int i = 0; i < V; i++) {
      for (int j = 0; j < V; j++) {

        if (adjacent(i, j) == 0) continue;
        const Eigen::Matrix3d& Ri = true_rotations.at(i);
        const Eigen::Matrix3d& Rj = true_rotations.at(j);

        Eigen::Matrix3d noise = noise_rotation.block(3 * i, 3 * j, 3, 3);
        if (i > j)
          measured_rotation.block(3 * i, 3 * j, 3, 3) = Ri.transpose() * Rj * noise;
        else
          measured_rotation.block(3 * i, 3 * j, 3, 3) = noise * Ri.transpose() * Rj;
      }
    }
  };

  Eigen::MatrixXd getTildeR() const { return measured_rotation; }

  const int V;
  const double noise_gain;

  int adjacent(int i, int j) const
  {
    return adjacent_graph(i, j);
  }

  Eigen::Matrix3d truth(int i) const
  {
    return true_rotations.at(i);
  }

private:
  Eigen::MatrixXd adjacent_graph;
  Eigen::MatrixXd noise_rotation;
  Eigen::MatrixXd measured_rotation;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> true_rotations;
};
