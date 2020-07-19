#pragma once
#include "g2o/so3.hpp"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>


class VertexRotation : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexRotation() {}
  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }


  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual void oplusImpl(const double* update)
  {
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
  }
};

class EdgeRotation : public g2o::BaseBinaryEdge<1, Eigen::Matrix3d, VertexRotation, VertexRotation>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRotation() {}
  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }

  void computeError()
  {
    const VertexRotation* vp0 = static_cast<const VertexRotation*>(vertex(0));
    const VertexRotation* vp1 = static_cast<const VertexRotation*>(vertex(1));
    _error(0) = (so3::exp(vp0->estimate()) * measurement() - so3::exp(vp1->estimate())).norm();
  }

  virtual void linearizeOplus()
  {
    VertexRotation* vp0 = static_cast<VertexRotation*>(vertex(0));
    VertexRotation* vp1 = static_cast<VertexRotation*>(vertex(1));
    Eigen::Matrix3d R0 = so3::exp(vp0->estimate());
    Eigen::Matrix3d R1 = so3::exp(vp1->estimate());
    Eigen::Matrix3d R01 = measurement();

    Eigen::Matrix3d Ri = -R1 * R01.transpose() * R0.transpose();
    Eigen::Matrix3d Rj = -R0 * R01 * R1.transpose();

    _jacobianOplusXi(0, 0) = -Ri(1, 2) + Ri(2, 1);
    _jacobianOplusXi(0, 1) = Ri(0, 2) - Ri(2, 0);
    _jacobianOplusXi(0, 2) = -Ri(0, 1) + Ri(1, 0);

    _jacobianOplusXj(0, 0) = -Rj(1, 2) + Rj(2, 1);
    _jacobianOplusXj(0, 1) = Rj(0, 2) - Rj(2, 0);
    _jacobianOplusXj(0, 2) = -Rj(0, 1) + Rj(1, 0);
  }
};
