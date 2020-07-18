#pragma once
#include "types.hpp"
#include <Eigen/Dense>
#include <iostream>

class RotationAveraging
{
public:
  const size_t V;

  RotationAveraging(size_t V);

  void optimize();
  void optimizeOnce();

  void setMeasurement(size_t from, size_t to, const Eigen::Matrix3d& R);
  bool getMeasurement(size_t from, size_t to, Eigen::Matrix3d& R) const;

  void setAbsolute(const Matrix3dVector& Rs);
  bool getAbsolute(size_t at, Eigen::Matrix3d& R) const;

  double getError(size_t from, size_t to) const;
  double getTotalError() const;

private:
  Eigen::MatrixXd tilde_r;
  Eigen::MatrixXd y;

  // the result of eliminating the k th row and column from Y^t
  Eigen::MatrixXd calcB(const Eigen::MatrixXd& Y);

  // the result of eliminating the k th column and all but the k th row from \tilde{R}
  Eigen::MatrixXd calcW(const Eigen::MatrixXd& tilde_R);

  // S_k = -B_k W_k [ (W_k^\top B_k W_k)^{\frac12} ]^\dag
  Eigen::MatrixXd calcS(const Eigen::MatrixXd& Bk, const Eigen::MatrixXd& Wk);

  // Y = [ I   S_k^\top]
  //     [ S_k B_k     ]
  Eigen::MatrixXd calcY(const Eigen::MatrixXd& S, const Eigen::MatrixXd& B);

  // warp
  void warp(Eigen::MatrixXd& tilde_R, Eigen::MatrixXd& Y);
};