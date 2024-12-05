// Copyright [2021] Optimus Ride Inc.

#pragma once

#include "altro/eigentypes.hpp"
#include "altro/problem/dynamics.hpp"

namespace altro {
namespace models {

/**
 * @brief Simple kinematic model of a unicycle / differential drive robot
 *
 * Has 3 states and 2 controls.
 *
 * # Mathematical Formulation
 * \f[
 * \begin{bmatrix} \dot{x} \\ \dot{y} \dot{\theta}  \end{bmatrix} =
 * \begin{bmatrix} v \cos(\theta) \\ v \sin(\theta) \\ \omega \end{bmatrix}
 * \f]
 *
 * where
 * \f[
 * u = \begin{bmatrix} v \\ \omega \end{bmatrix}
 * \f]
 */
class MobileManipulator_base_y : public problem::DiscreteDynamics {
 public:
  MobileManipulator_base_y() = default;
  static constexpr int NStates = 6;
  static constexpr int NControls = 1;
  int StateDimension() const override { return NStates; }
  int ControlDimension() const override { return NControls; }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> out) override;
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<MatrixXd> jac) override;
  void Hessian(const VectorXdRef& x, const VectorXdRef& u, const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) override;
  bool HasHessian() const override { return true; };

    // base function
  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, float t, float h,
                        Eigen::Ref<VectorXd> xdot)override
  {Evaluate(x, u, xdot);};
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u, float t, float h, Eigen::Ref<MatrixXd> jac)override
  {Jacobian(x, u, jac);};
  void Hessian(const VectorXdRef& x, const VectorXdRef& u, float t, float h, const VectorXdRef& b,
                       Eigen::Ref<MatrixXd> hess)override
  {Hessian(x, u, b, hess);};
};

}  // namespace examples
}  // namespace altro