// Copyright [2021] Optimus Ride Inc.

#include "Altro_control/Altro_model_joint1.hpp"

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

void MobileManipulator_joint1::Evaluate(const Eigen::Matrix4d& init_T , const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> out){
  
  //use x build 4*4 matrix
  Eigen::Matrix4d T0;
 T0 << cos(x[4])* cos(x[5]), cos(x[5])* sin(x[3])* sin(x[4]) - cos(x[3])* sin(x[5]), cos(x[3])* cos(x[5])* sin(x[4]) + sin(x[3])* sin(x[5]), x[0],
       cos(x[4])* sin(x[5]), cos(x[3])* cos(x[5]) + sin(x[3])* sin(x[4])* sin(x[5]), cos(x[3])* sin(x[4])* sin(x[5]) - cos(x[5])* sin(x[3]), x[1],
       -sin(x[4]), cos(x[4])* sin(x[3]), cos(x[3])* cos(x[4]), x[2],
      0, 0, 0, 1;
  //build static transform matrix
  Eigen::Matrix4d T1;
  T1 << 1, 0, 0, 0.2625,
        0, 1, 0, 0,
        0, 0, 1, 0.2745,
        0, 0, 0, 1;
  Eigen::Matrix4d T2;
  T2 << 1, 0, 0, 0.0,
        0, -1, 0, 0,
        0, 0, -1, 0.1564,
        0, 0, 0, 1;
  //build transform matrix
  Eigen::Matrix4d T3;
  T3 << cos(u[0]*t), -sin(u[0]*t), 0, 0,
        sin(u[0]*t), cos(u[0]*t), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  //calculate the final transform matrix
  Eigen::Matrix4d T = T0*T1*init_T*T3;
  // get the roation matrix from T
  Eigen::Matrix<double,3,3> R;
  R << T(0,0), T(0,1), T(0,2),
       T(1,0), T(1,1), T(1,2),
       T(2,0), T(2,1), T(2,2);
  Eigen::Vector3d euler =rotationMatrixToRPY(R);
  out[0] = T(0,3);
  out[1] = T(1,3);
  out[2] = T(2,3);
  out[3] = euler[0];
  out[4] = euler[1];
  out[5] = euler[2];
  //print the input x and out
//   std::cout << "joint1 inputx T0" << T0 << std::endl;
//   std::cout << "joint1 inputx T0*T1" << T0*T1 << std::endl;
//   std::cout << "joint1 inputx T0*T1*T2" << T0*T1*T_init << std::endl;
//   std::cout << "joint1 inputx T0*T1*T2*T3" << T << std::endl;
//   std::cout << "joint1 inputx T0 E" << rotationMatrixToRPY(T0.block(0,0,3,3)) << std::endl;
//   std::cout << "joint1 inputx T0*T1 E" << rotationMatrixToRPY((T0*T1).block(0,0,3,3)) << std::endl;
//   std::cout << "joint1 inputx T0*T1*T2 E" << rotationMatrixToRPY((T0*T1*T_init).block(0,0,3,3) ) << std::endl;
//   std::cout << "joint1 inputx T0*T1*T2*T3 E" << euler << std::endl;
//   std::cout << "joint1 inputx T_init" << init_T << std::endl;
//   std::cout << "joint1 inputx" << x <<std::endl;
//   std::cout << "joint1 outputx" << out <<std::endl;
}

void MobileManipulator_joint1::Jacobian(const Eigen::Matrix4d& init_T ,const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<MatrixXd> jac) {
  auto start = std::chrono::high_resolution_clock::now();
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
  T_initAD << init_T(0,0) , init_T(0,1), init_T(0,2), init_T(0,3),
              init_T(1,0) , init_T(1,1), init_T(1,2), init_T(1,3),
              init_T(2,0) , init_T(2,1), init_T(2,2), init_T(2,3),
              init_T(3,0) , init_T(3,1), init_T(3,2), init_T(3,3);
    Eigen::Matrix <CppAD::AD<double>,4,4>  T1;
  T1 << 1, 0, 0, 0.2625,
        0, 1, 0, 0,
        0, 0, 1, 0.2745,
        0, 0, 0, 1;
  Eigen::Matrix <CppAD::AD<double>,4,4> T2;
  T2 << 1, 0, 0, 0.0,
        0, -1, 0, 0,
        0, 0, -1, 0.1564,
        0, 0, 0, 1;
  Eigen::Matrix <CppAD::AD<double>,4,4>  T0;
 T0 << cos(XU[4])* cos(XU[5]), cos(XU[5])* sin(XU[3])* sin(XU[4]) - cos(XU[3])* sin(XU[5]), cos(XU[3])* cos(XU[5])* sin(XU[4]) + sin(XU[3])* sin(XU[5]), XU[0],
       cos(XU[4])* sin(XU[5]), cos(XU[3])* cos(XU[5]) + sin(XU[3])* sin(XU[4])* sin(XU[5]), cos(XU[3])* sin(XU[4])* sin(XU[5]) - cos(XU[5])* sin(XU[3]), XU[1],
       -sin(XU[4]), cos(XU[4])* sin(XU[3]), cos(XU[3])* cos(XU[4]), XU[2],
      0, 0, 0, 1;
  //build transform matrix
  Eigen::Matrix <CppAD::AD<double>,4,4>T3;
  T3 << cos(XU[6]*t), -sin(XU[6]*t), 0, 0,
        sin(XU[6]*t), cos(XU[6]*t), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  //calculate the final transform matrix
  Eigen::Matrix <CppAD::AD<double>,4,4> T = T0*T1*T_initAD*T3;
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
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
      // std::cout << "joint1 elapsed time: " << elapsed.count() << " s\n";
//   for (int i = 36; i < 42; i++) {
//     jac(i-36,6) = jac_vector_AB[i];
//   }
  // give jac_vector to jac
//   for (int i = 0; i < 6; i++) {
//     jac(i,0) = jac_vector[i];
//   }
//   //print the jacobian VEctor
      // for (int i = 0; i < 36; i++) {
      //       std::cout << jac_vector_A[i] << std::endl;
      // }
      // for (int i = 0; i < 6; i++) {
      //       std::cout << jac_vector_B[i] << std::endl;
      // }
// //print the shape of jac
      // std::cout << jac << std::endl;
//       std::cout << jac.cols() << std::endl;

}

void MobileManipulator_joint1::Hessian(const VectorXdRef& x, const VectorXdRef& u, const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  ALTRO_UNUSED(b);
  hess.setZero();
}

}  // namespace examples
}  // namespace altro


//(0,0)  cos(x5)*cos(x6)
//(0,1) cos(x6)*sin(x4)*sin(x5) - cos(x4)*sin(x6)
//(0,2) sin(x4)*sin(x6) + cos(x4)*cos(x6)*sin(x5)
//(0,3) x1
//(1,0) cos(x5)*sin(x6)
//(1,1) cos(x4)*cos(x6) + sin(x4)*sin(x5)*sin(x6)
//(1,2) cos(x4)*sin(x5)*sin(x6) - cos(x6)*sin(x4)
//(1,3) x2
//(2,0) -sin(x5)
//(2,1) cos(x5)*sin(x4)
//(2,2) cos(x4)*cos(x5)
//(2,3) x3
//(3,0) 0
//(3,1) 0
//(3,2) 0
//(3,3) 1
//Eigen::Matrix4d T0;
//  T0 << cos(x[4])* cos(x[5]), cos(x[5])* sin(x[3])* sin(x[4]) - cos(x[3])* sin(x[5]), cos(x[3])* cos(x[5])* sin(x[4]) + sin(x[3])* sin(x[5]), x[0],
//        cos(x[4])* sin(x[5]), cos(x[3])* cos(x[5]) + sin(x[3])* sin(x[4])* sin(x[5]), cos(x[3])* sin(x[4])* sin(x[5]) - cos(x[5])* sin(x[3]), x[1],
//        -sin(x[4]), cos(x[4])* sin(x[3]), cos(x[3])* cos(x[4]), x[2],
//    0, 0, 0, 1;


