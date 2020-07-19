#pragma once
#include "core/types.hpp"
#include "g2o/g2o_types.hpp"
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

class G2oRotationAveraging
{
public:
  const size_t V;
  G2oRotationAveraging(size_t V);

  void setMeasurement(size_t from, size_t to, const Eigen::Matrix3d& R);
  bool getMeasurement(size_t from, size_t to, Eigen::Matrix3d& R) const;

  void setAbsolute(const Matrix3dVector& Rs);
  bool getAbsolute(size_t at, Eigen::Matrix3d& R) const;

  double getError(size_t from, size_t to) const;
  double getTotalSquaredError() const;

  void optimize(int max_iteration);

private:
  void initializeOptimizer();
  int pair2int(int from, int to) const;
  EdgeRotation* findEdge(size_t from, size_t to) const;

  bool is_optimizer_initialized;

  g2o::SparseOptimizer optimizer;
};
