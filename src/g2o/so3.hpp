#pragma once
#include <Eigen/Dense>

namespace so3
{
Eigen::Matrix3d exp(const Eigen::Vector3d& w);
Eigen::Matrix3d hat(const Eigen::Vector3d& w);

Eigen::Vector3d log(const Eigen::Matrix3d& R);
Eigen::Vector3d dehat(const Eigen::Matrix3d& R);

}  // namespace so3