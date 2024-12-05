#pragma once

#include <iostream>
#include <cmath>
//add eigen
#include <Eigen/Dense>
#include "cppad/cppad.hpp"
#include <tuple>
// 用于存储Roll, Pitch, Yaw角度的结构体
struct RPY {
    double roll;
    double pitch;
    double yaw;
};

// 函数：将旋转矩阵转换为Roll, Pitch, Yaw
Eigen::Matrix<double,3,1> rotationMatrixToRPY(Eigen::Matrix<double,3,3> mat);

Eigen::Matrix<CppAD::AD<double>,3,1> rotationMatrixToRPYAD(Eigen::Matrix<CppAD::AD<double>,3,3> mat);
Eigen::Matrix<double, 3, 1> rotationMatrixToRPYtest(const Eigen::Matrix<double, 3, 3>& R);

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  return q.w() * qRef.vec() - qRef.w() * q.vec() + q.vec().cross(qRef.vec());
}

/**
 * Compute the quaternion distance measure jacobian
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x4 matrix representing the quaternion distance jacobian.
 * The columns are the partial derivatives of [q.x, q.y, q,z, qw] (default Eigen order)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 4> quaternionDistanceJacobian(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  Eigen::Matrix<SCALAR_T, 3, 4> jacobian;
  // clang-format off
  jacobian << -qRef.w(), qRef.z(), -qRef.y(), qRef.x(),
              -qRef.z(), -qRef.w(), qRef.x(), qRef.y(),
              qRef.y(), -qRef.x(), -qRef.w(), qRef.z();
  // clang-format on
  return jacobian;
}

Eigen::Matrix3d RPYtoRotation_Matrix(double roll, double pitch, double yaw) ;

std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d> RPYtoRotation_MatrixJacobian(double roll, double pitch, double yaw);
Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);
Eigen::Matrix<double,4,3> quaternionJacobian(double roll, double pitch, double yaw); 