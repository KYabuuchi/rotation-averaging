#include "g2o/so3.hpp"

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


}  // namespace so3