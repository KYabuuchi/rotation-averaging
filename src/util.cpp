#define _USE_MATH_DEFINES
#include "util.hpp"
#include <cmath>
#include <random>

namespace util
{

namespace
{
std::mt19937 mt;
}

// 次数Nの隣接グラフを生成する
Eigen::MatrixXd initCyclicAdjacentGraph(int N)
{
  Eigen::MatrixXd graph = Eigen::MatrixXd::Zero(N, N);
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      if (std::abs(i - j) == 1) graph(i, j) = 1;
      if (std::abs(i - j) == N - 1) graph(i, j) = 1;
    }
  }
  return graph;
}


// 2次元球面(S2)上で一様乱択
Eigen::Vector3d randomS2()
{
  std::uniform_real_distribution<> rand1(0, 1);
  double z = rand1(mt);
  double theta = M_PI * rand1(mt);
  double r = std::sqrt(1 - z * z);
  return Eigen::Vector3d(r * std::cos(theta), r * std::sin(theta), z);
}

// 回転角に制限を設定できるランダムな回転行列
Eigen::Matrix3d noiseRotation(double max = M_PI / 10.0)
{
  std::uniform_real_distribution<> rand1(-1, 1);
  Eigen::Vector3d e = randomS2();
  double theta = max * rand1(mt);
  return Eigen::AngleAxisd(theta, e).toRotationMatrix();
}

// 回転角に制限を設定できるランダムな回転行列
Eigen::Matrix3d noiseZRotation(double max = M_PI / 10.0)
{
  std::uniform_real_distribution<> rand1(-1, 1);
  double theta = max * rand1(mt);
  return Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Vector3d eulerAngles(const Eigen::Matrix3d& R)
{
  return R.eulerAngles(2, 1, 0) * 180.0 / M_PI;
}

Eigen::Matrix3d randomRotation()
{
  return Eigen::Quaterniond::UnitRandom().toRotationMatrix();
}

Eigen::MatrixXd pseudeInverse(const Eigen::MatrixXd& A)
{
  auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd::Identity(A.cols(), A.rows());
  const auto& singularValues = svd.singularValues();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    if (singularValues(i) > 1e-6) {
      singularValuesInv(i, i) = 1 / singularValues(i);
    } else {
      singularValuesInv(i, i) = 0;
    }
  }
  return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

bool isSemiDefinite(const Eigen::MatrixXd& A)
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(A);
  Eigen::MatrixXd rt = solver.operatorSqrt();
  return std::isfinite(rt(0));
}

// https://mathoverflow.net/questions/86539/closest-3d-rotation-matrix-in-the-frobenius-norm-sense
Eigen::Matrix3d normalize(const Eigen::Matrix3d& R)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d Vt = svd.matrixV().transpose();
  if (R.determinant() < 0) {
    return -U * Vt;
  }

  return U * Vt;
}
}  // namespace util