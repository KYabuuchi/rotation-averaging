#include "g2o/g2o_ra.hpp"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>

G2oRotationAveraging::G2oRotationAveraging(size_t V) : V(V)
{
  // Setup the solver
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> MyBlockSolver;
  typedef g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType> MyLinearSolver;
  optimizer.setVerbose(false);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  optimizer.setAlgorithm(solver);


  // Setup the vertices
  for (size_t i = 0; i < V; i++) {
    VertexRotation* vp = new VertexRotation;
    vp->setId(i);
    optimizer.addVertex(vp);
  }

  is_optimizer_initialized = false;
}

int G2oRotationAveraging::pair2int(int from, int to) const
{
  int hash1 = std::hash<int>{}(from);
  int hash2 = std::hash<int>{}(to);

  // https://stackoverflow.com/questions/4948780/magic-number-in-boosthash-combine
  int seed = 0;
  seed ^= hash1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= hash2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

void G2oRotationAveraging::setMeasurement(size_t from, size_t to, const Eigen::Matrix3d& R)
{
  EdgeRotation* ep = new EdgeRotation;
  ep->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  ep->setVertex(0, optimizer.vertices().find(from)->second);
  ep->setVertex(1, optimizer.vertices().find(to)->second);
  ep->setId(pair2int(from, to));
  ep->setMeasurement(R);
  optimizer.addEdge(ep);
  is_optimizer_initialized = false;
}

EdgeRotation* G2oRotationAveraging::findEdge(size_t from, size_t to) const
{
  EdgeRotation* ep = nullptr;
  auto itr = optimizer.edges().begin();
  for (; itr != optimizer.edges().cend(); itr++) {
    if ((*itr)->id() == pair2int(from, to)) {
      ep = dynamic_cast<EdgeRotation*>(*itr);
    }
  }
  return ep;
}


bool G2oRotationAveraging::getMeasurement(size_t from, size_t to, Eigen::Matrix3d& R) const
{
  EdgeRotation* ep = findEdge(from, to);
  if (ep == nullptr)
    return true;

  R = ep->measurement();
  return false;
}

void G2oRotationAveraging::setAbsolute(const Matrix3dVector& Rs)
{
  for (size_t i = 0; i < V; i++) {
    dynamic_cast<VertexRotation*>(optimizer.vertices().find(i)->second)->setEstimate(so3::log(Rs.at(i)));
  }
}

bool G2oRotationAveraging::getAbsolute(size_t at, Eigen::Matrix3d& R) const
{
  if (at >= V)
    return true;

  Eigen::Vector3d r = dynamic_cast<VertexRotation*>(optimizer.vertices().find(at)->second)->estimate();
  R = so3::exp(r);
  return false;
}

double G2oRotationAveraging::getError(size_t from, size_t to) const
{
  double error = -1;
  EdgeRotation* ep = findEdge(from, to);
  if (ep != nullptr) {
    ep->computeError();
    error = ep->error()(0);
  }
  return error;
}

double G2oRotationAveraging::getTotalError() const
{
  double total_error = 0;
  for (size_t i = 0; i < V; i++) {
    for (size_t j = 0; j < V; j++) {
      double error = getError(i, j);
      if (error < 0) continue;
      total_error += error;
    }
  }
  return total_error;
}

void G2oRotationAveraging::initializeOptimizer()
{
  optimizer.initializeOptimization();
  is_optimizer_initialized = true;
}

void G2oRotationAveraging::optimize(int max_iteration)
{
  if (!is_optimizer_initialized)
    initializeOptimizer();

  optimizer.optimize(max_iteration);
}