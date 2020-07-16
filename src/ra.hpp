#pragma once
#include <Eigen/Dense>
#include <iostream>

class RotationAveraging
{
public:
  const size_t V;

  RotationAveraging(size_t V) : V(V)
  {
    tilde_r.setZero(3 * V, 3 * V);
    y.setIdentity(3 * V, 3 * V);
  }

  void setMeasurement(size_t from, size_t to, const Eigen::Matrix3d& R)
  {
    tilde_r.block(3 * from, 3 * to, 3, 3) = R;
    tilde_r.block(3 * to, 3 * from, 3, 3) = R.transpose();
  }

  void optimize()
  {
    for (int i = 0; i < V; i++) {
      Eigen::MatrixXd Bk = calcB(y);        // 3(N-1) x 3(N-1)
      Eigen::MatrixXd Wk = calcW(tilde_r);  // 3(N-1) x 3
      Eigen::MatrixXd Sk = calcS(Bk, Wk);   // 3(N-1) x 3
      y = calcY(Sk, Bk);                    // 3N     x 3N
      warp(tilde_r, y);
    }
  }

  bool getMeasurement(size_t from, size_t to, Eigen::Matrix3d& R) const
  {
    R = tilde_r.block(3 * from, 3 * to, 3, 3);
    return R.isZero();
  }

  Eigen::Matrix3d getAbsolute(size_t at) const
  {
    return y.block(0, 3 * at, 3, 3);
  }

  double getError(size_t from, size_t to) const
  {
    Eigen::Matrix3d R_m;
    if (getMeasurement(from, to, R_m))
      return -1;

    Eigen::Matrix3d R_t, R_f;
    R_f = y.block(0, 3 * from, 3, 3);
    R_t = y.block(0, 3 * to, 3, 3);

    return frobenius2radian((R_f.transpose() * R_t - R_m).norm());
  }

private:
  Eigen::MatrixXd tilde_r;
  Eigen::MatrixXd y;

  double frobenius2radian(double norm) const
  {
    return std::asin(norm / (2 * 1.41413)) * 2;
  }

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