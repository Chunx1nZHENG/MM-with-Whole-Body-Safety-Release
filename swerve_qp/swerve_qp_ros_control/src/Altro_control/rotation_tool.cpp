#include "Altro_control/rotation_tool.hpp"
#include <parallel/algorithmfwd.h>



// 函数：将旋转矩阵转换为Roll, Pitch, Yaw
Eigen::Matrix<double,3,1> rotationMatrixToRPY(Eigen::Matrix<double,3,3> mat){
    Eigen::Vector3d  rpy;

    // 计算Pitch角度，注意处理奇异点
    if (mat(2,0) >= 1) { 
        rpy[0] = -1* std::atan2(mat(0,1), mat(1,1));

        // 这里假设yaw为0
        rpy[1] = -M_PI / 2;

        // 计算Yaw角度
        rpy[2] = 0;
    } 
    else if (mat(2,0) <= -1) {
        rpy[0] = std::atan2(mat(0,1), mat(1,1));

        // 这里假设yaw为0
        rpy[1] = M_PI / 2;

        // 计算Yaw角度
        rpy[2] = 0;
    } else {
        // 计算Pitch角度
        rpy[0] = std::atan2(mat(2,1), mat(2,2));
        

        // 计算Roll角度
        rpy[1] = std::atan2(-mat(2,0),std::sqrt(mat(0,0)*mat(0,0) + mat(1,0)*mat(1,0)));

        // 计算Yaw角度
        rpy[2] = std::atan2(mat(1,0), mat(0,0));
    }

    return rpy;

}


Eigen::Matrix<CppAD::AD<double>,3,1> rotationMatrixToRPYAD(Eigen::Matrix<CppAD::AD<double>,3,3> mat){
    Eigen::Matrix<CppAD::AD<double>,3,1>  rpy;

    // compare with rotationMatrixToRPY function ,rewrite into cppad version
    // 计算Pitch角度，注意处理奇异点
    if (mat(2,0) >= 1) { 
        rpy[0] = -1* CppAD::atan2(mat(0,1), mat(1,1));

        // 这里假设yaw为0
        rpy[1] = -M_PI / 2;

        // 计算Yaw角度
        rpy[2] = 0;
    } 
    else if (mat(2,0) <= -1) {
        rpy[0] = CppAD::atan2(mat(0,1), mat(1,1));

        // 这里假设yaw为0
        rpy[1] = M_PI / 2;

        // 计算Yaw角度
        rpy[2] = 0;
    } else {
        // 计算Pitch角度
        rpy[0] = CppAD::atan2(mat(2,1), mat(2,2));
        

        // 计算Roll角度
        rpy[1] = CppAD::atan2(-mat(2,0),CppAD::sqrt(mat(0,0)*mat(0,0) + mat(1,0)*mat(1,0)));

        // 计算Yaw角度
        rpy[2] = CppAD::atan2(mat(1,0), mat(0,0));
    }
    
    return rpy;
}


Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    // 计算角度的一半
    double halfYaw = yaw * 0.5;
    double halfPitch = pitch * 0.5;
    double halfRoll = roll * 0.5;

    // 计算三角函数值
    double cosYaw = std::cos(halfYaw);
    double sinYaw = std::sin(halfYaw);
    double cosPitch = std::cos(halfPitch);
    double sinPitch = std::sin(halfPitch);
    double cosRoll = std::cos(halfRoll);
    double sinRoll = std::sin(halfRoll);

    // 根据RPY计算四元数的各个分量
    Eigen::Quaterniond quaternion;
    quaternion.x() = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; // x
    quaternion.y() = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; // y
    quaternion.z() = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; // z
    quaternion.w() = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; // w

    return quaternion;
}


Eigen::Matrix<double,4,3> quaternionJacobian(double roll, double pitch, double yaw) {
    // 计算角度的一半
    double halfYaw = yaw * 0.5;
    double halfPitch = pitch * 0.5;
    double halfRoll = roll * 0.5;

    // 计算三角函数值
    double cosYaw = std::cos(halfYaw);
    double sinYaw = std::sin(halfYaw);
    double cosPitch = std::cos(halfPitch);
    double sinPitch = std::sin(halfPitch);
    double cosRoll = std::cos(halfRoll);
    double sinRoll = std::sin(halfRoll);

    // 四元数的各个分量
    double x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    double y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    double z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    double w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

    // 计算关于roll的偏导数
    double dx_droll = 0.5 * (cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
    double dy_droll = -0.5 * (sinRoll * sinPitch * cosYaw - cosRoll * cosPitch * sinYaw);
    double dz_droll = -0.5 * (sinRoll * cosPitch * sinYaw + cosRoll * sinPitch * cosYaw);
    double dw_droll = 0.5 * (-sinRoll * cosPitch * cosYaw + cosRoll * sinPitch * sinYaw);

    // 计算关于pitch的偏导数
    double dx_dpitch = 0.5 * (-sinRoll * sinPitch * cosYaw - cosRoll * cosPitch * sinYaw);
    double dy_dpitch = 0.5 * (cosRoll * sinPitch * cosYaw - sinRoll * sinPitch * sinYaw);
    double dz_dpitch = 0.5 * (-cosRoll * sinPitch * sinYaw - sinRoll * cosPitch * cosYaw);
    double dw_dpitch = 0.5 * (-cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);

    double dx_dyaw = 0.5 * (-sinRoll * cosPitch * sinYaw - cosRoll * sinPitch * cosYaw);
    double dy_dyaw = 0.5 * (-cosRoll * sinPitch * sinYaw + sinRoll * cosPitch * cosYaw);
    double dz_dyaw = 0.5 * (cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
    double dw_dyaw = 0.5 * (-sinRoll * sinPitch * sinYaw + cosRoll * cosPitch * cosYaw);

    // 将偏导数组合成向量
    
    Eigen::Matrix<double,4,3> derivatives_matrix ;
    derivatives_matrix << dx_droll, dx_dpitch, dx_dyaw,
                          dy_droll, dy_dpitch, dy_dyaw,
                          dz_droll, dz_dpitch, dz_dyaw,
                          dw_droll, dw_dpitch, dw_dyaw;

    return derivatives_matrix;
}

Eigen::Matrix3d RPYtoRotation_Matrix(double roll, double pitch, double yaw) {
    Eigen::Matrix3d R_x;
    Eigen::Matrix3d R_y;
    Eigen::Matrix3d R_z;

    // 绕 x 轴旋转 (Roll)
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    // 绕 y 轴旋转 (Pitch)
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    // 绕 z 轴旋转 (Yaw)
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 总旋转矩阵 R = R_z * R_y * R_x
    return R_z * R_y * R_x;
}

std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d> RPYtoRotation_MatrixJacobian(double roll, double pitch, double yaw) {
    Eigen::Matrix3d R_x;
    Eigen::Matrix3d R_y;
    Eigen::Matrix3d R_z;

    // 绕 x 轴旋转 (Roll)
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    // 绕 y 轴旋转 (Pitch)
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    // 绕 z 轴旋转 (Yaw)
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    Eigen::Matrix3d dR_x_dRoll;
    dR_x_dRoll << 0, 0, 0,
                  0, -sin(roll), -cos(roll),
                  0, cos(roll), -sin(roll);
    Eigen::Matrix3d dR_y_dPitch;
    dR_y_dPitch << -sin(pitch), 0, cos(pitch),
                   0, 0, 0,
                   -cos(pitch), 0, -sin(pitch);
    
    Eigen::Matrix3d dR_z_dYaw;
    dR_z_dYaw << -sin(yaw), -cos(yaw), 0,
                 cos(yaw), -sin(yaw), 0,
                 0, 0, 0;

    Eigen::Matrix3d dR_dRoll = R_z * R_y * dR_x_dRoll;
    Eigen::Matrix3d dR_dPitch = R_z * dR_y_dPitch * R_x;
    Eigen::Matrix3d dR_dYaw = dR_z_dYaw * R_y * R_x;

    return std::make_tuple(dR_dRoll, dR_dPitch, dR_dYaw);
}