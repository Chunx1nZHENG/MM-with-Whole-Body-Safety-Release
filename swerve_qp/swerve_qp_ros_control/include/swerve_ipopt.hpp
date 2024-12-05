#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <tf2_ros/transform_listener.h>
#include "SwerveModelInfo.hpp"
#include <cppad/ipopt/solve.hpp>  
#include <cppad/ipopt/solve_result.hpp>
#include <cppad/cg.hpp>

constexpr double PI = 3.14159265;
namespace ocs2
{
    namespace swerve_qp_ipopt
    {
    using CppAD::AD;
    using ad_scalar_t = CppAD::AD<double>;
    using ad_vector_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1>;
    using ad_matrix_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
    class FG_eval {
        public:
            using ADvector = CppAD::vector<CppAD::AD<double>>;
            using Dvector =  CppAD::vector<scalar_t>;
            explicit FG_eval(size_t arm_dimension, PinocchioInterface& pinocchioInterface, swerve::SwerveModelInfo swerveModelInfo,vector_t upperBound,vector_t joint_upperBound,ros::NodeHandle& controller_nh);

            void operator()(ADvector& Fg, const ADvector& X);
            bool get_started(void);
            matrix_t get_jacobian(vector_t state);
            matrix_t get_joint_jacobian(vector_t state);
            vector_t get_error(vector_t state) ;
            scalar_t get_armAngle(vector_t state);
            void get_updatedState(vector_t state,vector_t desired_position, Eigen::Quaternion<scalar_t> desired_orientation);
            vector_t get_solution(vector_t state,vector_t desired_position, Eigen::Quaternion<scalar_t> desired_orientation);
            Dvector dVectorToDvector(const vector_t& adVec);
            vector_t baseKinemics(const vector_t& baseInput);
          
            bool AugmentedLagrangian_solver(Dvector xi, 
                                           Dvector xl, 
                                           Dvector xu, 
                                           Dvector gl, 
                                           Dvector gu, 
                                           FG_eval &fg_eval,
                                           CppAD::ipopt::solve_result<Dvector>& solution,
                                           bool print);

            bool AugmentedLagrangian_solver_noCPPAD(Dvector    xi, 
                                                Dvector     xl, 
                                                Dvector     xu, 
                                                Dvector     gl, 
                                                Dvector     gu, 
                                                FG_eval&    fg_eval,
                                                CppAD::ipopt::solve_result<Dvector>& solution,
                                                bool        print);
            void Set_currentState(vector_t state);

        private:
            size_t arm_dimension_;

             // Gain term (lambda) for control minimisation
            scalar_t Y_;
            // Quadratic component of objective function
            matrix_t Q_;
            // Linear component of objective function
            vector_t C_;
            scalar_t ps_;
            scalar_t pi_;

            vector_t desired_vector_;
            vector_t eeposition_;
            quaternion_t eeOrientation_;

            Eigen::Quaternion<scalar_t> desired_orientation_;
            vector_t desired_position_;
            vector_t upperBound_;
            vector_t joint_upperBound_;
            // Spatial error
            scalar_t et_ ;
            matrix_t jacobian_;
            // manipulability jacobian matrix
            vector_t jacobian_m_;
            scalar_t mt_;
            tf2_ros::Buffer buffer_;

            vector_t currState_;

            ad_vector_t dq_;

            vector_t gl_;
            vector_t gu_;
            vector_t xl_;
            vector_t xu_;

            PinocchioInterface& pinocchioInterface_;
            std::unique_ptr<PinocchioStateInputMapping<ocs2::scalar_t>> mappingPtr_;
            swerve::SwerveModelInfo swerveModelInfo_;

            std::string options_;
            CppAD::ipopt::solve_result<Dvector> solution_;

            scalar_t KpSteer_;
            scalar_t K_theta_;
            scalar_t K_vel_;
         
        };
        
        template<typename scalar>
        CppAD::vector<scalar> EigenMatrixToCppADVector(const Eigen::Matrix<scalar, Eigen::Dynamic, 1>& eigen_matrix) {
            CppAD::vector<scalar> cppad_vector(eigen_matrix.size());
            for(int i = 0; i < eigen_matrix.size(); ++i) {
                cppad_vector[i] = eigen_matrix(i);
            }
            return cppad_vector;
        }

        template<typename scalar>
        Eigen::Matrix<scalar, Eigen::Dynamic, 1> CppADVectorToEigenMatrix(const CppAD::vector<scalar>& cppad_vector) {
            Eigen::Matrix<scalar, Eigen::Dynamic, 1> eigen_matrix(cppad_vector.size());
            for(size_t i = 0; i < cppad_vector.size(); ++i) {
                eigen_matrix(i) = cppad_vector[i];
            }
            return eigen_matrix;
        }

    double normalizeAngle(double angle);
    };// namespace swerve_qp_control        
 } 
