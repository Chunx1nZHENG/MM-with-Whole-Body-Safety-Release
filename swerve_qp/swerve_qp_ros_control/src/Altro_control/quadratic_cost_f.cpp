// Copyright [2021] Optimus Ride Inc.

#include "Altro_control/quadratic_cost_f.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "Altro_control/rotation_tool.hpp"
namespace altro {
namespace costs {

double QuadraticCostF::Evaluate(const VectorXdRef& x,
                               const VectorXdRef& u) {
  // double cost = 0;
  // for (int i = 0; i < 3; ++i) {
  //   cost += 0.5 * (x(i) - xref_(i)) * Q_(i,i) * (x(i) - xref_(i));
  // }
  // // for (int i = 3; i < 6; ++i) {
  // //   double diff_abs = std::abs(x(i) - xref_(i));
  // //   //print the diff_abs
    
  // //   if (diff_abs > M_PI) {
  // //     diff_abs = 2 * M_PI - diff_abs;
  // //   }
  // //   cost += 0.5 * diff_abs * Q_(i,i) * diff_abs;
  // // }
  
  // return cost;
  return 0.5 * x.dot(Q_ * x) + q_.dot(x) + c_;
}

void QuadraticCostF::Gradient(const VectorXdRef& x,
                             const VectorXdRef& u, Eigen::Ref<VectorXd> dx,
                             Eigen::Ref<VectorXd> du) {
  // using CppAD::sin;
  // using CppAD::cos;
  // vector< AD<double> > X(6);
  // CppAD::Independent(X);
  // AD<double> cost = 0;
  // // use cppad to calculate the gradient
  // for (int i = 0; i < 3; ++i) {
  //   cost += 0.5 * (x(i) - xref_(i)) * Q_(i,i) * (x(i) - xref_(i));
  // }
  // for (int i = 3; i < 6; ++i) {
  //   AD<double> diff_abs = CppAD::abs(X[i] - xref_(i));
  //   if (diff_abs > M_PI) {
  //     diff_abs = 2 * M_PI - diff_abs;
  //   }
  //   cost += 0.5 * diff_abs * Q_(i,i) * diff_abs;
  // }
  // vector< AD<double> > Y(1);
  // Y[0] = cost;
  // CppAD::ADFun<double> f(X, Y);
  // vector<double> xvec(6);
  // for (int i = 0; i < 6; ++i) {
  //   xvec[i] = x(i);
  // }
  // vector<double> grad = f.Jacobian(xvec);
  // for (int i = 0; i < 6; ++i) {
  //   dx(i) = grad[i];
  // }
  //print the gradient
  // dx(0) = Q_(0,0) * x(0) + q_(0);
  // dx(1) = Q_(1,1) * x(1) + q_(1);
  // dx(2) = Q_(2,2) * x(2) + q_(2);
  // dx(3) = 0;
  // dx(4) = 0;
  // dx(5) = 0;
  // for (int i = 3; i < 6; ++i) {
  //   double diff = x(i) - xref_(i);
  //   double x_temp = xref_(i);
  //   if (diff > M_PI) {
  //     x_temp = x(i) - 2 * M_PI;
  //   } else if (diff < -M_PI) {
  //     x_temp = x(i) + 2 * M_PI;
  //   }
  //   dx(i) = Q_(i,i) * x(i) - Q_(i,i) * x_temp;
  // }

  dx = Q_ * x + q_;
  std::cout << "gradient: " << dx.transpose() << std::endl;
  du.setZero();
}
void QuadraticCostF::Hessian(const VectorXdRef& x,
                            const VectorXdRef& u, Eigen::Ref<MatrixXd> dxdx,
                            Eigen::Ref<MatrixXd> dxdu, Eigen::Ref<MatrixXd> dudu) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  dxdx = Q_;
  dudu.setZero();
  dxdu.setZero();
}

void QuadraticCostF::Validate() {
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



double QuadraticCostF_test::Evaluate(const VectorXdRef& x,
                               const VectorXdRef& u) {

  double cost = 0;
  for (int i = 0; i < 3; ++i) {
    cost += 0.5 * (x(i) - xref_(i)) * Q_(i,i) * (x(i) - xref_(i));
  }
  VectorXd rpy = x.segment<3>(3);
  VectorXd rpy_ref = xref_.segment<3>(3);

  Eigen::Quaternion q = rpyToQuaternion(rpy(0), rpy(1), rpy(2));
  Eigen::Quaternion q_ref = rpyToQuaternion(rpy_ref(0), rpy_ref(1), rpy_ref(2));
  cost +=  0.5 * quaternionDistance(q, q_ref).transpose() * (Q_.block(3,3,3,3) * quaternionDistance(q, q_ref));
  return cost;
}

void QuadraticCostF_test::Gradient(const VectorXdRef& x,
                             const VectorXdRef& u, Eigen::Ref<VectorXd> dx,
                             Eigen::Ref<VectorXd> du) {
  dx.resize(6);
  dx.segment<3>(0) = (Q_.block(0,0,3,3) * (x.segment<3>(0) - xref_.segment<3>(0)));

  VectorXd rpy = x.segment<3>(3);
  VectorXd rpy_ref = xref_.segment<3>(3);
  Eigen::Quaternion q = rpyToQuaternion(rpy(0), rpy(1), rpy(2));
  Eigen::Quaternion q_ref = rpyToQuaternion(rpy_ref(0), rpy_ref(1), rpy_ref(2));

  dx.segment<3>(3) =  quaternionJacobian(rpy(0), rpy(1), rpy(2)).transpose() * quaternionDistanceJacobian(q, q_ref).transpose() * Q_.block(3,3,3,3) * quaternionDistance(q, q_ref);
  du.setZero();
}


void QuadraticCostF_test::Hessian(const VectorXdRef& x,
                            const VectorXdRef& u, Eigen::Ref<MatrixXd> dxdx,
                            Eigen::Ref<MatrixXd> dxdu, Eigen::Ref<MatrixXd> dudu) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  dxdx = Q_;
  dudu.setZero();
  dxdu.setZero();
}

void QuadraticCostF_test::Validate() {
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
