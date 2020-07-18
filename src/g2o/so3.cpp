#include "g2o/so3.hpp"
#include <cmath>

namespace so3
{
Eigen::Matrix3d exp(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d hat_w = hat(w);
  float norm = w.norm();
  if (norm < 1e-6)
    return Eigen::Matrix3d::Identity();

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R += std::sin(norm) / norm * hat_w;
  R += (1 - std::cos(norm)) / (norm * norm) * hat_w * hat_w;
  return R;
}

Eigen::Matrix3d hat(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d hat_w;
  // clang-format off
    hat_w << 
         0, -w(2),  w(1),
      w(2),     0, -w(0),
     -w(1),  w(0),     0;
  // clang-format on
  return hat_w;
}

Eigen::Vector3d log(const Eigen::Matrix3d& R)
{
  double tmp = std::min(std::max(R.trace() * 0.5 - 0.5, -1.0), 1.0);
  double phi = std::acos(tmp);
  if (std::abs(phi) < 1e-6)
    return dehat((R - R.transpose()) * 0.5);
  return dehat(phi * (R - R.transpose()) * 0.5 / std::sin(phi));
}

Eigen::Vector3d dehat(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d w;
  w << R(2, 1), -R(2, 0), R(1, 0);
  return w;
}


}  // namespace so3