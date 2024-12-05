// Copyright [2021] Optimus Ride Inc.

#include "Altro_control/Altro_model_base_y.hpp"

#include <cmath>

#include "altro/utils/utils.hpp"

#define t  1

namespace altro {
namespace models {

void MobileManipulator_base_y::Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> out){
  // out[0] = x[0];
  // out[1] = x[1]+T*u[0];
  // out[2] = x[2];
  // out[3] = x[3];
  // out[4] = x[4];
  // out[5] = x[5];
  out[0] = x[0]-u[0]*t*sin(x[5]);
  out[1] = x[1]+u[0]*t*cos(x[5]);
  out[2] = x[2];
  out[3] = x[3];
  out[4] = x[4];
  out[5] = x[5];
}

void MobileManipulator_base_y::Jacobian(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<MatrixXd> jac) {
  jac(0, 0) = 1;
  jac(0, 5) = -t*u[0]*cos(x[5]);
  jac(1, 1) = 1;
  jac(1, 5) = -t*u[0]*sin(x[5]);
  jac(2, 2) = 1;
  jac(3, 3) = 1;
  jac(4, 4) = 1;
  jac(5, 5) = 1;
  jac(0, 6) = -t*sin(x[5]);
  jac(1, 6) = t*cos(x[5]);
}

void MobileManipulator_base_y::Hessian(const VectorXdRef& x, const VectorXdRef& u, const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  ALTRO_UNUSED(b);
  hess.setZero();
}

}  // namespace examples
}  // namespace altro