// Cyclicな観測のみ対応
#pragma once
#include "types.hpp"
#include "util.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

class ProblemGenerator
{
public:
  const int V;
  const double noise_gain;

  ProblemGenerator(const int V, const double noise_gain = 0.2) : V(V), noise_gain(noise_gain)
  {
    // Construct the adjacent graph
    adjacent_graph.setZero(V, V);
    adjacent_graph(V - 1, 0) = 1;
    for (int i = 0; i < V - 1; i++)
      adjacent_graph(i, i + 1) = 1;
    std::cout << "adjacent graph=\n\033[1;31m"
              << adjacent_graph << "\033[0m" << std::endl;


    // Generate true rotation matrix
    true_rotations.push_back(Eigen::Matrix3d::Identity());
    for (int i = 1; i < V; i++) {
      true_rotations.push_back(util::randomRotation());
    }


    // Initialize random noise rotation matrix
    for (size_t i = 0; i < V; i++) {
      for (size_t j = 0; j < V; j++) {
        if (!isAdjacent(i, j)) continue;

        Eigen::Matrix3d N = util::noiseRotation(noise_gain);
        noise_rotations.emplace(std::make_pair(i, j), N);
        noise_rotations.emplace(std::make_pair(j, i), N.transpose());
      }
    }

    // Initialize measured rotation matrix
    for (size_t i = 0; i < V; i++) {
      for (size_t j = 0; j < V; j++) {
        if (!isAdjacent(i, j)) continue;

        const Eigen::Matrix3d& Ri = true_rotations.at(i);
        const Eigen::Matrix3d& Rj = true_rotations.at(j);
        const Eigen::Matrix3d& N = noise_rotations.at(std::make_pair(i, j));

        measured_rotations.emplace(std::make_pair(i, j), Ri.transpose() * Rj * N);
      }
    }
  };

  bool isAdjacent(int i, int j) const
  {
    return adjacent_graph(i, j) > 0;
  }

  Eigen::Matrix3d truth(int i) const
  {
    return true_rotations.at(i);
  }

  Eigen::Matrix3d measured(size_t from, size_t to) const
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    if (!isAdjacent(from, to)) return R;
    return measured_rotations.at(std::make_pair(from, to));
  }

  const RelativeRotations& getMeasurement() const
  {
    return measured_rotations;
  }

private:
  Eigen::MatrixXd adjacent_graph;
  RelativeRotations noise_rotations;
  RelativeRotations measured_rotations;
  Matrix3dVector true_rotations;
};
