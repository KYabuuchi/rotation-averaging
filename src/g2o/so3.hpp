#pragma once
#include <Eigen/Dense>

namespace so3
{
Eigen::Matrix3d exp(const Eigen::Vector3d& w);
Eigen::Matrix3d hat(const Eigen::Vector3d& w);

}  // namespace so3