#include "Altro_control/Altro_problem.hpp"

using namespace ocs2;
namespace altro {
namespace problems {

MobileManipulatorProblem::MobileManipulatorProblem() {
}

problem::Problem MobileManipulatorProblem::MakeProblem(const bool add_constraints, const bool track_attitude) {
    problem::Problem prob(N);

    tf = 0.1;
    float h = GetTimeStep();
    Eigen::VectorXd xi = Eigen::VectorXd::Zero(6);
    // xf << 0.0, 1.4, 1.2, 0,1.5, 0.0;
    // xi << 1.0, 1.0, 0.0, 0.0, 0.0, 0.5;
    
    lb = {-q_bnd};
    ub = {+q_bnd};
    lb_v = {-v_bnd};
    ub_v = {+v_bnd};
    lb_x =  {-3.2, -2.1, -2.4, -3.2, -1.9, -3.2};
    ub_x =  {3.2, 2.1, 2.4, 3.2, 1.9, 3.2};
    lb_vyaw = {-vyaw_bnd};
    ub_vyaw = {+vyaw_bnd};
    std::cout << "Qi: test" << std::endl;
    // ca = Eigen::MatrixXd::Zero(6, 6);
    Qi(0,0) = 200;
    Qi(1,1) = 200;
    std::cout << "Qi: test1" << std::endl;
    // Qi(5,5) = 100;
    Q.diagonal().setConstant(0.0);
    R.diagonal().setConstant(0.5);
    
    A.resize(8);
    b.resize(8);
    F.resize(8);
    g.resize(8);
    c.resize(8);
    if (!track_attitude) {
    // // if (true) {
            Qf.diagonal().setConstant(4);
            Qf(3,3) = 0;
            Qf(4,4) = 0;
            Qf(5,5) = 0;
            Qi(0,0) = 200;
            Qi(1,1) = 200;
            Qi(2,2) = 0;

    }
    else
    {
        Qf.diagonal().setConstant(40);
        Qf(3,3) = 10;
        Qf(4,4) = 10;
        Qf(5,5) = 10;
        Qi(0,0) = 0;
        Qi(1,1) = 0;
        Qi(2,2) = 0;
    }

    // robot link shapes
    for (int k = 0; k < N-1; ++k) {
        A[k] = Eigen::MatrixXd::Zero(6, 3);
        b[k] = Eigen::VectorXd::Zero(6);
        A[k] <<  1, 0  , 0,
                -1, 0  , 0,
                 0, 1  , 0,
                 0, -1 , 0,
                 0, 0  , 1,
                 0, 0  , -1;
    }   
    b[0] << 0.381, 0.381, 0.306, 0.306, 0.26, 0;
    b[1] << 0.0465, 0.0465, 0.0465, 0.0465, 0.15, 0;
    b[2] << 0.0465, 0.0465, 0.0465, 0.0465, 0, 0.08;
    b[3] << 0.0465, 0.0465, 0, 0.35, 0.034, 0.034;
    b[4] << 0.0465, 0.0465, 0.21, 0, 0.0425, 0.0425;
    b[5] << 0.0465, 0.0465, 0.0425, 0.0425, 0, 0.08;
    b[6] << 0.0355, 0.0355, 0.42, 0.0, 0.0445, 0.0445;
    b[7] << 0.0355, 0.0355, 0.0445, 0.0445, 0.0, 0.09;
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, 0.5*R, xf, uref));
    // qcostt = std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Qi, 0.1*R, base_target, uref));
    // prob.SetCostFunction(qcostt, 0);
    // // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, 0.5*R, xf, uref));
    // qcostt = std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Qi, 0.1*R, base_target, uref));
    // prob.SetCostFunction(qcostt, 1);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, 10*R, xf, uref));
    // prob.SetCostFunction(qcost, 2);
    // // qcostt = std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Qi, R, xi, uref));
    // // prob.SetCostFunction(qcostt, 2);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    // prob.SetCostFunction(qcost, 3);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    // prob.SetCostFunction(qcost, 4);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    // prob.SetCostFunction(qcost, 5);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    // prob.SetCostFunction(qcost, 6);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    // prob.SetCostFunction(qcost, 7);
    // qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    // prob.SetCostFunction(qcost, 8);
    // // qterm = std::make_shared<costs::QuadraticCostF>(costs::QuadraticCostF::LQRCost(Qf, R, xf, uref,true));
    // qterm = std::make_shared<costs::QuadraticCostF_test>(costs::QuadraticCostF_test::LQRCost(Qf, 0.1*R, xf, uref,true));
    // prob.SetCostFunction(qterm, N);
    qcosts.clear();
    qcosts.resize(N+1, nullptr);

    qcosts[0] = std::make_shared<costs::sumCost>();
    qcostt = std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Qi, 0.1*R, base_target, uref));
    qcosts[0]->AddCost(qcostt);

    qcosts[1] = std::make_shared<costs::sumCost>();
    qcostt = std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Qi, 0.1*R, base_target, uref));
    qcosts[1]->AddCost(qcostt);

    qcosts[2] = std::make_shared<costs::sumCost>();
    qcostt = std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Qi, 1000*R, base_target, uref));
    qcosts[2]->AddCost(qcostt);

    qcosts[3] = std::make_shared<costs::sumCost>();
    qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    qcosts[3]->AddCost(qcost);

    qcosts[4] = std::make_shared<costs::sumCost>();
    qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    qcosts[4]->AddCost(qcost);

    qcosts[5] = std::make_shared<costs::sumCost>();
    qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    qcosts[5]->AddCost(qcost);

    qcosts[6] = std::make_shared<costs::sumCost>();
    qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    qcosts[6]->AddCost(qcost);

    qcosts[7] = std::make_shared<costs::sumCost>();
    qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    qcosts[7]->AddCost(qcost);

    qcosts[8] = std::make_shared<costs::sumCost>();
    qcost = std::make_shared<costs::QuadraticCostU>(costs::QuadraticCostU::LQRCost(Q, R, xf, uref));
    qcosts[8]->AddCost(qcost);

    qcosts[9] = std::make_shared<costs::sumCost>();
    qterm = std::make_shared<costs::QuadraticCostF_test>(costs::QuadraticCostF_test::LQRCost(Qf, R, xf, uref,true));
    qcosts[9]->AddCost(qterm);

  

    //Dynamics
    // for (int k = 0; k < N; ++k) {
    // prob.SetDynamics(std::make_shared<ModelType>(model), k);
    // }
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_base_x>(MobileManipulator_base_x), 0);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_base_y>(MobileManipulator_base_y), 1);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_base_yaw>(MobileManipulator_base_yaw), 2);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_joint1>(MobileManipulator_joint1), 3);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_joint2>(MobileManipulator_joint2), 4);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_joint3>(MobileManipulator_joint3), 5);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_joint4>(MobileManipulator_joint4), 6);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_joint5>(MobileManipulator_joint5), 7);
    prob.SetDynamics(std::make_shared<altro::models::MobileManipulator_joint6>(MobileManipulator_joint6), 8);
    
    


    // Constraints
    prob.SetConstraint(std::make_shared<altro::examples::ControlBound>(lb_v, ub_v), 0);
    prob.SetConstraint(std::make_shared<altro::examples::ControlBound>(lb_v, ub_v), 1);
    prob.SetConstraint(std::make_shared<altro::examples::ControlBound>(lb_vyaw, ub_vyaw), 2);
    for (int k = 3; k < N; ++k) {
        prob.SetConstraint(std::make_shared<altro::examples::ControlBound>(lb, ub), k);
    }


    for(int k = 3; k < N; ++k) {
        std::vector<double> lower_bound = {lb_x[k-3]};
        std::vector<double> upper_bound = {ub_x[k-3]};
        std::vector<double> arm_state;
        for (int i = 0; i < 6; ++i) {
            arm_state.push_back(state(i));
        }
    
        prob.SetConstraint(std::make_shared<altro::examples::StateBound>(lower_bound, upper_bound, arm_state, k), k);
    }
    // std::shared_ptr<altro::examples::PointConstraints> point_constraints;

    // point_constraints = std::make_shared<altro::examples::PointConstraints>("s0", A[0], b[0], F[0], g[0], c[0]);
    // for (int k = 0; k < point_constraints->getPointConstraints().size(); ++k) {
    //     prob.SetConstraint(point_constraints->getPointConstraints()[k], 1);
    // }
    // point_constraints = std::make_shared<altro::examples::PointConstraints>("s1", A[0], b[0], F[0], g[0], c[0]);
    // for (int k = 0; k < point_constraints->getPointConstraints().size(); ++k) {
    //     prob.SetConstraint(point_constraints->getPointConstraints()[k], 2);
    // }
    // point_constraints = std::make_shared<altro::examples::PointConstraints>("s2", A[0], b[0], F[0], g[0], c[0]);
    // for (int k = 0; k < point_constraints->getPointConstraints().size(); ++k) {
    //     prob.SetConstraint(point_constraints->getPointConstraints()[k], 3);
    // }
    // point_constraints = std::make_shared<altro::examples::PointConstraints>("s4", A[3], b[3], F[3], g[3], c[3]);
    // for (int k = 0; k < point_constraints->getPointConstraints().size(); ++k) {
    //     prob.SetConstraint(point_constraints->getPointConstraints()[k], 5);
    // }
    // point_constraints = std::make_shared<altro::examples::PointConstraints>("s5", A[4], b[4], F[4], g[4], c[4]);
    // for (int k = 0; k < point_constraints->getPointConstraints().size(); ++k) {
    //     prob.SetConstraint(point_constraints->getPointConstraints()[k], 6);
    // }
    // point_constraints = std::make_shared<altro::examples::PointConstraints>("s7", A[6], b[6], F[6], g[6], c[6]);
    // for (int k = 0; k < point_constraints->getPointConstraints().size(); ++k) {
    //     prob.SetConstraint(point_constraints->getPointConstraints()[k], 8);
    // }

    // prob.SetConstraint(std::make_shared<examples::GoalConstraint>(xf), N);
    
    // Initial State
    prob.SetInitialState(x0);

    
    prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp0", A[0], b[0], F[0], g[0], c[0]), 1);
    prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp1", A[0], b[0], F[0], g[0], c[0]), 2);
    prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp2", A[0], b[0], F[0], g[0], c[0]), 3);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp3", A[2], b[2], F[0], g[0], c[0]), 4); // TODO: 可能可以去除
    prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp4", A[3], b[3], F[3], g[3], c[3]), 5);
    prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp5", A[4], b[4], F[4], g[4], c[4]), 6);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp6", A[5], b[5], F[0], g[0], c[0]), 7);
    prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp7", A[6], b[6], F[6], g[6], c[6]), 8);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPConstraint>("sdp8", A[7], b[7], F[0], g[0], c[0]), 9);


    // prob.SetConstraint(std::make_shared<altro::examples::SDPCBFonstraint>("sdp0", A[0], b[0], F[0], g[0], c[0]), 1);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPCBFonstraint>("sdp1", A[0], b[0], F[0], g[0], c[0]), 2);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPCBFonstraint>("sdp2", A[0], b[0], F[0], g[0], c[0]), 3);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPCBFonstraint>("sdp4", A[3], b[3], F[0], g[0], c[0]), 5);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPCBFonstraint>("sdp5", A[4], b[4], F[0], g[0], c[0]), 6);
    // prob.SetConstraint(std::make_shared<altro::examples::SDPCBFonstraint>("sdp7", A[6], b[6], F[0], g[0], c[0]), 8);


    // std::shared_ptr<costs::SDPRelaxedBarrierPenalty> sdp_cost;
    // sdp_cost = std::make_shared<costs::SDPRelaxedBarrierPenalty>("sdp0", A[0], b[0], F[0], g[0], c[0]);
    // qcosts[1]->AddCost(sdp_cost);
    // sdp_cost = std::make_shared<costs::SDPRelaxedBarrierPenalty>("sdp1", A[0], b[0], F[0], g[0], c[0]);
    // qcosts[2]->AddCost(sdp_cost);
    // sdp_cost = std::make_shared<costs::SDPRelaxedBarrierPenalty>("sdp2", A[0], b[0], F[0], g[0], c[0]);
    // qcosts[3]->AddCost(sdp_cost);
    // sdp_cost = std::make_shared<costs::SDPRelaxedBarrierPenalty>("sdp4", A[3], b[3], F[3], g[3], c[3]);
    // qcosts[5]->AddCost(sdp_cost);
    // sdp_cost = std::make_shared<costs::SDPRelaxedBarrierPenalty>("sdp5", A[4], b[4], F[4], g[4], c[4]);
    // qcosts[6]->AddCost(sdp_cost);
    // sdp_cost = std::make_shared<costs::SDPRelaxedBarrierPenalty>("sdp7", A[5], b[5], F[5], g[5], c[5]);
    // qcosts[7]->AddCost(sdp_cost);
    
    for(int k = 0; k <= N; ++k) {
        prob.SetCostFunction(qcosts[k], k);
    }

    return prob;
}

}
}