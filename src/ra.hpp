#pragma once
#include <Eigen/Dense>

namespace ra
{
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

}  // namespace ra