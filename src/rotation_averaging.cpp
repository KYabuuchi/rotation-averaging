#define _USE_MATH_DEFINES
#include "rotation_averaging.hpp"
#include "util.hpp"
#include <cmath>

RotationAveraging::RotationAveraging(size_t V) : V(V)
{
  tilde_r.setZero(3 * V, 3 * V);
  y.setIdentity(3 * V, 3 * V);
}

void RotationAveraging::setAbsolute(size_t at, const Eigen::Matrix3d& R)
{
}
bool RotationAveraging::getAbsolute(size_t at, Eigen::Matrix3d& R) const
{
  if (at >= V)
    return true;
  R = util::normalize(y.block(0, 3 * at, 3, 3));
  return false;
}


void RotationAveraging::setMeasurement(size_t from, size_t to, const Eigen::Matrix3d& R)
{
  tilde_r.block(3 * from, 3 * to, 3, 3) = R;
  tilde_r.block(3 * to, 3 * from, 3, 3) = R.transpose();
}

void RotationAveraging::optimize()
{
  for (int i = 0; i < V; i++) {
    Eigen::MatrixXd Bk = calcB(y);        // 3(N-1) x 3(N-1)
    Eigen::MatrixXd Wk = calcW(tilde_r);  // 3(N-1) x 3
    Eigen::MatrixXd Sk = calcS(Bk, Wk);   // 3(N-1) x 3
    y = calcY(Sk, Bk);                    // 3N     x 3N
    warp(tilde_r, y);
  }
}

bool RotationAveraging::getMeasurement(size_t from, size_t to, Eigen::Matrix3d& R) const
{
  R = tilde_r.block(3 * from, 3 * to, 3, 3);
  return R.isZero();
}

double RotationAveraging::getError(size_t from, size_t to) const
{
  Eigen::Matrix3d R_m;
  if (getMeasurement(from, to, R_m))
    return -1;

  Eigen::Matrix3d R_t, R_f;
  R_f = y.block(0, 3 * from, 3, 3);
  R_t = y.block(0, 3 * to, 3, 3);

  return (R_f.transpose() * R_t - R_m).norm();
}

double RotationAveraging::getTotalError() const
{
  double total_error = 0;
  for (size_t i = 0; i < V - 1; i++) {
    for (size_t j = i + 1; j < V; j++) {
      double error = getError(i, j);
      if (error < 0) continue;
      total_error += error;
    }
  }
  return total_error;
}


// the result of eliminating the k th row and column from Y^t
Eigen::MatrixXd RotationAveraging::calcB(const Eigen::MatrixXd& Y)
{
  int cols = Y.cols();
  return Y.bottomRightCorner(cols - 3, cols - 3);
}

// the result of eliminating the k th column and all but the k th row from \tilde{R}
Eigen::MatrixXd RotationAveraging::calcW(const Eigen::MatrixXd& tilde_R)
{
  int cols = tilde_R.cols();
  return tilde_R.bottomLeftCorner(cols - 3, 3);
}

// S_k = -B_k W_k [ (W_k^\top B_k W_k)^{\frac12} ]^\dag
Eigen::MatrixXd RotationAveraging::calcS(const Eigen::MatrixXd& Bk, const Eigen::MatrixXd& Wk)
{
  Eigen::MatrixXd WtBW = Wk.transpose() * Bk * Wk;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(WtBW);
  Eigen::MatrixXd sqrtWtBW = solver.operatorSqrt();

  // Wとして-tilde{R}を使っているためここで
  // - Bk * Wk * (sqrtWtBW.inverse());
  // となるべき符号が反転する
  return Bk * Wk * (sqrtWtBW.inverse());
}

// Y = [ I   S_k^\top]
//     [ S_k B_k     ]
Eigen::MatrixXd RotationAveraging::calcY(const Eigen::MatrixXd& S, const Eigen::MatrixXd& B)
{
  int cols = S.cols() + B.cols();
  Eigen::MatrixXd Y = Eigen::MatrixXd::Identity(cols, cols);
  Y.block(3, 0, cols - 3, 3) = S;
  Y.block(0, 3, 3, cols - 3) = S.transpose();
  Y.block(3, 3, cols - 3, cols - 3) = B;
  return Y;
}

void RotationAveraging::warp(Eigen::MatrixXd& tilde_R, Eigen::MatrixXd& Y)
{
  int cols = tilde_R.cols();
  Eigen::MatrixXd y = Y.block(0, 0, cols, 3);
  Y.block(0, 0, cols - 3, cols - 3) = Y.block(3, 3, cols - 3, cols - 3);
  Y.topRightCorner(cols - 3, 3) = y.bottomRows(cols - 3);
  Y.bottomLeftCorner(3, cols - 3) = y.bottomRows(cols - 3).transpose();

  Eigen::MatrixXd r = tilde_R.block(0, 0, cols, 3);
  tilde_R.block(0, 0, cols - 3, cols - 3) = tilde_R.block(3, 3, cols - 3, cols - 3);
  tilde_R.topRightCorner(cols - 3, 3) = r.bottomRows(cols - 3);
  tilde_R.bottomLeftCorner(3, cols - 3) = r.bottomRows(cols - 3).transpose();
}
