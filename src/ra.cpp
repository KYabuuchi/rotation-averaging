#define _USE_MATH_DEFINES
#include "ra.hpp"
#include <cmath>

namespace ra
{
Eigen::MatrixXd getRTilde(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2, const Eigen::Matrix3d& R3)
{
  Eigen::MatrixXd R_tilde = Eigen::MatrixXd::Zero(9, 9);
  R_tilde.block(0, 3, 3, 3) = R1.transpose() * R2;  // 1->2
  R_tilde.block(0, 6, 3, 3) = R1.transpose() * R3;  // 1->3
  R_tilde.block(3, 0, 3, 3) = R2.transpose() * R1;  // 2->1
  R_tilde.block(3, 6, 3, 3) = R2.transpose() * R3;  // 2->3
  R_tilde.block(6, 0, 3, 3) = R3.transpose() * R1;  // 3->1
  R_tilde.block(6, 3, 3, 3) = R3.transpose() * R2;  // 3->2
  return R_tilde;
}

// the result of eliminating the k th row and column from Y^t
Eigen::MatrixXd calcB(const Eigen::MatrixXd& Y)
{
  int cols = Y.cols();
  return Y.bottomRightCorner(cols - 3, cols - 3);
}

// the result of eliminating the k th column and all but the k th row from \tilde{R}
Eigen::MatrixXd calcW(const Eigen::MatrixXd& tilde_R)
{
  int cols = tilde_R.cols();
  return tilde_R.bottomLeftCorner(cols - 3, 3);
}

// S_k = -B_k W_k [ (W_k^\top B_k W_k)^{\frac12} ]^\dag
Eigen::MatrixXd calcS(const Eigen::MatrixXd& Bk, const Eigen::MatrixXd& Wk)
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
Eigen::MatrixXd calcY(const Eigen::MatrixXd& S, const Eigen::MatrixXd& B)
{
  int cols = S.cols() + B.cols();
  Eigen::MatrixXd Y = Eigen::MatrixXd::Identity(cols, cols);
  Y.block(3, 0, cols - 3, 3) = S;
  Y.block(0, 3, 3, cols - 3) = S.transpose();
  Y.block(3, 3, cols - 3, cols - 3) = B;
  return Y;
}

void warp(Eigen::MatrixXd& tilde_R, Eigen::MatrixXd& Y)
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

}  // namespace ra