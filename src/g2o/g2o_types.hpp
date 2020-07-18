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

  // Analytical Jacobian
  // virtual void linearizeOplus()
  // {
  // }
};
