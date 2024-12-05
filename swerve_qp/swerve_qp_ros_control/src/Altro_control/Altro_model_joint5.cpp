// Copyright [2021] Optimus Ride Inc.

#include "Altro_control/Altro_model_joint5.hpp"

#include <cmath>

#include "altro/utils/utils.hpp"
#include "modernrobotics/modern_robotics.h"
#include "cppad/cppad.hpp"
#include "Altro_control/rotation_tool.hpp"
#define t  1
using CppAD::AD;   // use AD as abbreviation for CppAD::AD
using std::vector; // use vector as abbreviation for std::vector

namespace altro {
namespace models {

void MobileManipulator_joint5::Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> out){
//   std::cout << "joint5" << std::endl;  
  //use x build 4*4 matrix
  Eigen::Matrix4d T0;
 T0 << cos(x[4])* cos(x[5]), cos(x[5])* sin(x[3])* sin(x[4]) - cos(x[3])* sin(x[5]), cos(x[3])* cos(x[5])* sin(x[4]) + sin(x[3])* sin(x[5]), x[0],
       cos(x[4])* sin(x[5]), cos(x[3])* cos(x[5]) + sin(x[3])* sin(x[4])* sin(x[5]), cos(x[3])* sin(x[4])* sin(x[5]) - cos(x[5])* sin(x[3]), x[1],
       -sin(x[4]), cos(x[4])* sin(x[3]), cos(x[3])* cos(x[4]), x[2],
        0, 0, 0, 1;
  //build static transform matrix
  Eigen::Matrix4d T1;
  T1 << 1, 0, 0, 0,
        0, 0, 1, -0.0002,
        0, -1, 0, -0.1059,
        0, 0, 0, 1;
  //build transform matrix
  Eigen::Matrix4d T2;
  T2 << cos(u[0]*t), -sin(u[0]*t), 0, 0,
        sin(u[0]*t), cos(u[0]*t), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  //calculate the final transform matrix
  Eigen::Matrix4d T = T0*T_init*T2;
  // get the roation matrix from T
  Eigen::Matrix<double,3,3> R;
  R << T(0,0), T(0,1), T(0,2),
       T(1,0), T(1,1), T(1,2),
       T(2,0), T(2,1), T(2,2);
  Eigen::Vector3d euler = rotationMatrixToRPY(R);
  out[0] = T(0,3);
  out[1] = T(1,3);
  out[2] = T(2,3);
  out[3] = euler[0];
  out[4] = euler[1];
  out[5] = euler[2];        
   
}

void MobileManipulator_joint5::Jacobian(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<MatrixXd> jac) {
  using CppAD::sin;
  using CppAD::cos;

  //build a cppad vector to present the x,use eigen
  vector< AD<double> > XU(7);
//  
  XU[6] = u[0];
  XU[0] = x[0]; XU[1] = x[1]; XU[2] = x[2]; XU[3] = x[3]; XU[4] = x[4]; XU[5] = x[5];
  //build static transform matrix

  CppAD::Independent(XU);
  vector< AD<double> > X(6);
  Eigen::Matrix <CppAD::AD<double>,4,4>  T_initAD;
  T_initAD << T_init(0,0) , T_init(0,1), T_init(0,2), T_init(0,3),
              T_init(1,0) , T_init(1,1), T_init(1,2), T_init(1,3),
              T_init(2,0) , T_init(2,1), T_init(2,2), T_init(2,3),
              T_init(3,0) , T_init(3,1), T_init(3,2), T_init(3,3);
  Eigen::Matrix <CppAD::AD<double>,4,4>  T1;
  T1 << 1, 0, 0, 0,
        0, 0, 1, -0.0002,
        0, -1, 0, -0.1059,
        0, 0, 0, 1;
  Eigen::Matrix <CppAD::AD<double>,4,4>  T0;
 T0 << cos(XU[4])* cos(XU[5]), cos(XU[5])* sin(XU[3])* sin(XU[4]) - cos(XU[3])* sin(XU[5]), cos(XU[3])* cos(XU[5])* sin(XU[4]) + sin(XU[3])* sin(XU[5]), XU[0],
       cos(XU[4])* sin(XU[5]), cos(XU[3])* cos(XU[5]) + sin(XU[3])* sin(XU[4])* sin(XU[5]), cos(XU[3])* sin(XU[4])* sin(XU[5]) - cos(XU[5])* sin(XU[3]), XU[1],
       -sin(XU[4]), cos(XU[4])* sin(XU[3]), cos(XU[3])* cos(XU[4]), XU[2],
      0, 0, 0, 1;
  //build transform matrix
  Eigen::Matrix <CppAD::AD<double>,4,4>T2;
  T2 << cos(XU[6]*t), -sin(XU[6]*t), 0, 0,
        sin(XU[6]*t), cos(XU[6]*t), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  //calculate the final transform matrix
  Eigen::Matrix <CppAD::AD<double>,4,4> T = T0*T_initAD*T2;
  Eigen::Matrix <CppAD::AD<double>,3,1> position = T.block(0,3,3,1);
  Eigen::Matrix <CppAD::AD<double>,3,3> R ;
      R << T(0,0), T(0,1), T(0,2),
      T(1,0), T(1,1), T(1,2),
      T(2,0), T(2,1), T(2,2);
  Eigen::Matrix <CppAD::AD<double>,3,1> euler = rotationMatrixToRPYAD(R);
  X[0] = T(0,3);
  X[1] = T(1,3);
  X[2] = T(2,3);
  X[3] = euler[0];
  X[4] = euler[1];
  X[5] = euler[2];

  CppAD::ADFun<double> f(XU, X);
  //calculate the jacobian matrix
  vector<double> jac_vector_AB(6*7);
  vector<double> xu_vector(7);
  xu_vector[6] = u[0];
  for (size_t i = 0; i < 6; i++)
  {
      xu_vector[i] = x[i];
  }
  
  jac_vector_AB = f.Jacobian(xu_vector);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 7; j++) {
      jac(i,j) = jac_vector_AB[i*7+j];
    }
  }
}

void MobileManipulator_joint5::Hessian(const VectorXdRef& x, const VectorXdRef& u, const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  ALTRO_UNUSED(b);
  hess.setZero();
}

}  // namespace examples
}  // namespace altro