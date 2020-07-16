#pragma once
#include <Eigen/Dense>

namespace util
{
// 回転行列間の距離
double calcAngleResidual(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2);

// 2次元球面(S2)上での一様乱数
Eigen::Vector3d randomS2();

// 回転角に制限を設定できるランダムな回転行列
Eigen::Matrix3d noiseRotation(double max);

// 回転角に制限を設定できるランダムな回転行列
Eigen::Matrix3d noiseZRotation(double max);

// 回転行列->Euler角
Eigen::Vector3d eulerAngles(const Eigen::Matrix3d& R);

// 一様な回転行列
Eigen::Matrix3d randomRotation();

// 疑似逆行列
Eigen::MatrixXd pseudeInverse(const Eigen::MatrixXd& A);

// 半正定値性のチェック
bool isSemiDefinite(const Eigen::MatrixXd& A);

// Fノルムで最近傍の回転行列に正規化
Eigen::Matrix3d normalize(const Eigen::Matrix3d& R);

}  // namespace util