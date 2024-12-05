// Copyright [2021] Optimus Ride Inc.

#include "Altro_control/quadratic_cost_u.hpp"

namespace altro {
namespace costs {

double QuadraticCostU::Evaluate(const VectorXdRef& x,
                               const VectorXdRef& u) {
  //print the value of the cost function
  //std::cout << "Q: " << Q_ << std::endl;
  return 0.5 * u.dot(R_ * u) +  r_.dot(u) + c_;
}

void QuadraticCostU::Gradient(const VectorXdRef& x,
                             const VectorXdRef& u, Eigen::Ref<VectorXd> dx,
                             Eigen::Ref<VectorXd> du) {
  dx.setZero();
  du = R_ * u + r_;
}

void QuadraticCostU::Hessian(const VectorXdRef& x,
                            const VectorXdRef& u, Eigen::Ref<MatrixXd> dxdx,
                            Eigen::Ref<MatrixXd> dxdu, Eigen::Ref<MatrixXd> dudu) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  dxdx.setZero();
  dudu = R_;
  dxdu.setZero();
}

void QuadraticCostU::Validate() {
  ALTRO_ASSERT(Q_.rows() == n_, "Q has the wrong number of rows");
  ALTRO_ASSERT(Q_.cols() == n_, "Q has the wrong number of columns");
  ALTRO_ASSERT(R_.rows() == m_, "R has the wrong number of rows");
  ALTRO_ASSERT(R_.cols() == m_, "R has the wrong number of columns");
  ALTRO_ASSERT(H_.rows() == n_, "H has the wrong number of rows");
  ALTRO_ASSERT(H_.cols() == m_, "H has the wrong number of columns");

  // Check symmetry of Q and R
  ALTRO_ASSERT(Q_.isApprox(Q_.transpose()), "Q is not symmetric");
  ALTRO_ASSERT(R_.isApprox(R_.transpose()), "R is not symmetric");

  // Check that R is positive definite
  if (!terminal_) {
    Rfact_.compute(R_);
    ALTRO_ASSERT(Rfact_.info() == Eigen::Success, "R must be positive definite");
  }

  // Check if Q is positive semidefinite
  Qfact_.compute(Q_);
  ALTRO_ASSERT(Qfact_.info() == Eigen::Success,
               "The LDLT decomposition could of Q could not be computed. "
               "Must be positive semi-definite");
  Eigen::Diagonal<const MatrixXd> D = Qfact_.vectorD();
  bool ispossemidef = true;
  (void) ispossemidef; // surpress erroneous unused variable error
  for (int i = 0; i < n_; ++i) {
    if (D(i) < 0) {
      ispossemidef = false;
      break;
    }
  }
  ALTRO_ASSERT(ispossemidef, "Q must be positive semi-definite");
}


}  // namespace examples
}  // namespace altro