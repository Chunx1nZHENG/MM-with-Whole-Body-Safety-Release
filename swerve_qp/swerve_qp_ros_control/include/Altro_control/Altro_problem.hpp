#pragma once

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <ocs2_core/misc/LoadData.h>
#include "altro/eigentypes.hpp"
#include "altro/problem/problem.hpp"

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/augmented_lagrangian/al_problem.hpp"
#include "altro/common/trajectory.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "altro/problem/discretized_model.hpp"
#include "./Altro_model_base_yaw.hpp"
#include "./Altro_model_base_x.hpp"
#include "./Altro_model_base_y.hpp"
#include "./Altro_model_joint1.hpp"
#include "./Altro_model_joint2.hpp"
#include "./Altro_model_joint3.hpp"
#include "./Altro_model_joint4.hpp"
#include "./Altro_model_joint5.hpp"
#include "./Altro_model_joint6.hpp"
#include "./unicycle.hpp"
#include "Altro_control/quadratic_cost.hpp"
#include "Altro_control/quadratic_cost_f.hpp"
#include "Altro_control/quadratic_cost_u.hpp"
#include "Altro_control/basic_constraints.hpp"
#include "Altro_control/RelaxBarrierPenalty.hpp"
// #include "Altro_control/SDP_constraints.hpp"
#include <decomp_ros_utils/data_ros_utils.h>

namespace altro {
namespace problems {

    class MobileManipulatorProblem {
        public:
            static constexpr int NStates = 6;
            static constexpr int NControls = 1;

            MobileManipulatorProblem();
            // using ModelType = altro::problem::DiscreteDynamics;
            // using ModelType = altro::problem::DiscretizedModel<altro::examples::Unicycle>;
            // using ModelType2 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_base_y>;
            // using ModelType3 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_base_yaw>;
            // using ModelType4 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_joint1>;
            // using ModelType5 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_joint2>;
            // using ModelType6 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_joint3>;
            // using ModelType7 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_joint4>;
            // using ModelType8 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_joint5>;
            // using ModelType9 = altro::problem::DiscretizedModel<altro::models::MobileManipulator_joint6>;
	        using CostFunType = altro::examples::QuadraticCost;

            //Problem Data
            static constexpr int HEAP = Eigen::Dynamic;
            const int n = NStates;
            const int m = NControls;
            // ModelType model = ModelType(altro::examples::Unicycle());
            int N = 9;
            altro::models::MobileManipulator_base_x MobileManipulator_base_x;
            altro::models::MobileManipulator_base_y MobileManipulator_base_y;
            altro::models::MobileManipulator_base_yaw MobileManipulator_base_yaw;
            altro::models::MobileManipulator_joint1 MobileManipulator_joint1;
            altro::models::MobileManipulator_joint2 MobileManipulator_joint2;
            altro::models::MobileManipulator_joint3 MobileManipulator_joint3;
            altro::models::MobileManipulator_joint4 MobileManipulator_joint4;
            altro::models::MobileManipulator_joint5 MobileManipulator_joint5;
            altro::models::MobileManipulator_joint6 MobileManipulator_joint6;
            // altro::models::MobileManipulator_base_x model_base_x;

            //build a 6x6 identity matrix
            Eigen::MatrixXd Q = Eigen::VectorXd::Constant(NStates, 1e-2).asDiagonal();
            Eigen::MatrixXd Qi = Eigen::VectorXd::Constant(NStates, 1e-2).asDiagonal();
            Eigen::MatrixXd R = Eigen::VectorXd::Constant(NControls, 1e-2).asDiagonal();
            Eigen::MatrixXd Qf = Eigen::VectorXd::Constant(NStates, 100).asDiagonal();
            Eigen::VectorXd xf = Eigen::VectorXd::Zero(NStates);
            Eigen::VectorXd x0 = Eigen::VectorXd::Zero(NStates);
            Eigen::VectorXd base_target = Eigen::VectorXd::Zero(NStates);
            Eigen::VectorXd u0 = Eigen::VectorXd::Constant(NControls, 0.1);
            Eigen::VectorXd uref = Eigen::VectorXd::Zero(NControls);
            Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
            std::shared_ptr<examples::QuadraticCost> qcostt;
            std::shared_ptr<costs::QuadraticCostU> qcost;
            std::shared_ptr<costs::QuadraticCostF_test> qterm;
            std::vector<std::shared_ptr<costs::sumCost>> qcosts;
            double q_bnd = 0.1; 
            std::vector<double> lb;
            std::vector<double> ub;

            double v_bnd = 0.5; 
            std::vector<double> lb_v;
            std::vector<double> ub_v;

            std::vector<double> lb_x;
            std::vector<double> ub_x;
            
            double vyaw_bnd = 0.1; 
            std::vector<double> lb_vyaw;
            std::vector<double> ub_vyaw;

            altro::problem::Problem MakeProblem(const bool add_constraints = true, const bool track_attitude = false);

            template <int n_size = NStates, int m_size = NControls>
            altro::Trajectory<n_size, m_size> InitialTrajectory();

            template <int n_size = NStates, int m_size = NControls>
            altro::ilqr::iLQR<n_size, m_size> MakeSolver();

            template <int n_size = NStates, int m_size = NControls>
            altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver();

            float GetTimeStep() const { return tf / N; }

              //build a strruct to store the Translate matrix
            struct TranslateMatrix {
                Eigen::Matrix4d T_base_link_to_x_arm_link;
                Eigen::Matrix4d T_x_arm_link_to_bicep_link;
                Eigen::Matrix4d T_bicep_link_to_forearm_link;
                Eigen::Matrix4d T_forearm_link_to_spherical_wrist_2_link;
                Eigen::Matrix4d T_spherical_wrist_2_link_to_ee_link;
                TranslateMatrix()
                    : T_base_link_to_x_arm_link(Eigen::Matrix4d::Zero()),
                    T_x_arm_link_to_bicep_link(Eigen::Matrix4d::Zero()),
                    T_bicep_link_to_forearm_link(Eigen::Matrix4d::Zero()),
                    T_forearm_link_to_spherical_wrist_2_link(Eigen::Matrix4d::Zero()),
                    T_spherical_wrist_2_link_to_ee_link(Eigen::Matrix4d::Zero()){}
            } TranslateMatrix_;
            
            void setFreeRegion(const std::vector<LinearConstraint3D>& LinearConstraints, const std::vector<Vec3f>& freeregionCenters)
            {
                //get the length of the LinearConstraints
                int n = LinearConstraints.size();
std::cout << "FreeRegion num: " << n << std::endl;
                F.resize(n);
                g.resize(n);
                c.resize(n);
                for (int k = 0; k < n; ++k) {
                    F[k] = LinearConstraints[k].A_;
                    g[k] = LinearConstraints[k].b_; 
                    c[k] = freeregionCenters[k];
                }
std::cout << "Set Down" << std::endl;
            }
            // void setObstacle(const MatrixXd& F, const VectorXd& g, const VectorXd& c)
            // {
            //     this->F = F;
            //     this->g = g;
            //     this->c = c;
            // }
            private:
            float tf = 0.025;
            std::vector<Eigen::MatrixXd> A;
            std::vector<Eigen::VectorXd> b;
            std::vector<Eigen::VectorXd> q;
            std::vector<Eigen::MatrixXd> F;
            std::vector<Eigen::VectorXd> g;
            std::vector<Eigen::VectorXd> c;
            
    };


    template <int n_size, int m_size>
    altro::Trajectory<n_size, m_size> MobileManipulatorProblem::InitialTrajectory() {
        altro::Trajectory<n_size, m_size> Z(n, m, N);
        for (int k = 0; k < N; ++k) {
            Z.Control(k) = u0;
        }
        float h = GetTimeStep(); 
        Z.SetUniformStep(h);
        return Z;
    }

    template <int n_size, int m_size>
    altro::ilqr::iLQR<n_size, m_size> MobileManipulatorProblem::MakeSolver()
    {
        altro::problem::Problem prob = MakeProblem();
        prob = altro::augmented_lagrangian::BuildAugLagProblem<n_size, m_size>(prob);

        altro::ilqr::iLQR<n_size, m_size> solver(prob);

        std::shared_ptr<altro::Trajectory<n_size, m_size>> traj_ptr =
            std::make_shared<altro::Trajectory<n_size, m_size>>(InitialTrajectory<n_size, m_size>());

        solver.SetTrajectory(traj_ptr);
        solver.Rollout();
        return solver;
    }

    template <int n_size, int m_size>
    altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
    MobileManipulatorProblem::MakeALSolver() {
        altro::problem::Problem prob = MakeProblem(true,false);
        altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
        solver_al.SetTrajectory(
            std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
        solver_al.GetiLQRSolver().Rollout();
        return solver_al;
    }
}
}