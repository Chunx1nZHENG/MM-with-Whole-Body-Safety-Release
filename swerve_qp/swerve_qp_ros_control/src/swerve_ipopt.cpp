#include <pinocchio/fwd.hpp>
#include <pinocchio//algorithm/kinematics-derivatives.hxx>
#include <pinocchio//algorithm/kinematics-derivatives.hpp>

# include <ocs2_core/Types.h>

# include <cppad/cg.hpp>
# include "swerve_ipopt.hpp"
# include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
# include <ocs2_pinocchio_interface/PinocchioInterface.h>
# include "SwerveModelInfo.hpp"
# include <ocs2_robotic_tools/common/AngularVelocityMapping.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "pinocchio/multibody/fcl.hpp"

#include <ocs2_robotic_tools/common/RotationTransforms.h>
# include <cppad/ipopt/solve.hpp>  
# include <cppad/cppad.hpp>

#include "SwervePinocchioMapping.hpp"


namespace ocs2
{
namespace swerve_qp_ipopt
{

FG_eval::FG_eval(size_t arm_dimension, PinocchioInterface& pinocchioInterface, swerve::SwerveModelInfo swerveModelInfo,vector_t upperBound,vector_t joint_upperBound,ros::NodeHandle& controller_nh) 
:   arm_dimension_(arm_dimension),
    pinocchioInterface_(pinocchioInterface),
    swerveModelInfo_(swerveModelInfo),
    upperBound_(upperBound),
    joint_upperBound_(joint_upperBound)
{
    mappingPtr_.reset(new swerve::SwervePinocchioMapping(swerveModelInfo_));
    ocs2::ad_base_t test;
    bool as = CppAD::IdenticalZero(test);

   
    controller_nh.param("qp_params/K_theta", K_theta_, 0.1);  //Orientation gain
    controller_nh.param("qp_params/mt", mt_, 0.0);//Velocity gain
    controller_nh.param("qp_params/Y", Y_, 0.1);//
    controller_nh.param("qp_params/KpSteer", KpSteer_, 0.5);//Steer gain

    K_vel_ = 1;//Velocity gain
    C_ = vector_t::Zero(arm_dimension_+9);
    
    jacobian_m_ = vector_t(arm_dimension_);
    ps_ = 0.1;
    pi_ = 0.9;

    xu_ = vector_t(arm_dimension_+9);
    xl_ = vector_t(arm_dimension_+9);
    xu_[0] = upperBound[2];
    xl_[0] = -upperBound[2];

    xu_[1] = upperBound[0];
    xl_[1] = -upperBound[0];

    xu_[2] = upperBound[1];
    xl_[2] = -upperBound[1];

    for (int i = 0; i < arm_dimension_; i++) {
        xu_[i+3] = upperBound[swerve::sh_rot_input_ind + i];
        xl_[i+3] = -upperBound[swerve::sh_rot_input_ind + i];
    }
    for (int i = 0; i < 6; i++) {
        xu_[i+arm_dimension_+3] = 10000;
        xl_[i+arm_dimension_+3] = -10000;
    }
    solution_.x.resize(arm_dimension_+9);
    gl_= vector_t(arm_dimension_+6);
    gu_= vector_t(arm_dimension_+6);

    jacobian_ = matrix_t::Zero(6, arm_dimension_+ 3 + 6);
    // turn off any printing
    options_ += "Integer print_level  2\n";
    options_ += "String  sb           yes\n";
    // maximum number of iterations
    options_ += "Integer max_iter     1000\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options_ += "Numeric tol          1e-6\n";
    // derivative testing
    options_ += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options_ += "Numeric point_perturbation_radius  0.1\n";

    options_ += "String hessian_constant yes\n";

    options_ += "east_square_init_duals\n";

}

void FG_eval::operator()(ADvector& fg, const ADvector& X) {
    auto x = CppADVectorToEigenMatrix(X);
    auto object = 0.5*(x.transpose()*(Q_.cast<ad_scalar_t>()*x))+ C_.cast<ad_scalar_t>().transpose()*x;
    fg[0] = object(0,0);
    
    ad_scalar_t temp=0.5;

    auto equal_constraint = jacobian_.cast<ad_scalar_t>()*x ;
    for (int i = 0; i < 6; i++) {
        fg[i+1] = equal_constraint[i] ;
    }
    // The inequality constraints for joint limit avoidance
    for (int i = 0; i < arm_dimension_; i++) {
        fg[6+i+1] = x[i+3] ;
    }
    return;
}

matrix_t FG_eval::get_jacobian(vector_t state) {
    using ad_quaternion_t = Eigen::Quaternion<ocs2::ad_scalar_t>;
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
    using ad_vector_t = ocs2::ad_vector_t;
    using ad_scalar_t = ocs2::ad_scalar_t;
    using ad_matrix_t = ocs2::ad_matrix_t;

    matrix_t jacobian = matrix_t::Zero(6, arm_dimension_+3);
  
    const pinocchio::Model& model = pinocchioInterface_.getModel();
    // const pinocchio::Data& data = pinocchioInterface_->getData();
    // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
    pinocchio::Data data = pinocchio::Data(pinocchioInterface_.getData());
    const auto q = mappingPtr_->getPinocchioJointPosition(state);
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD;

 
    int indeeLink = model.getFrameId( "end_effector_link");
    
    matrix_t J = matrix_t::Zero(6, model.nv);
    

    data = pinocchio::Data(pinocchioInterface_.getData());
    pinocchio::getFrameJacobian(model, data, indeeLink, rf, J);
    //theta
    jacobian.block(0,0,6,1)= J.block(0,5,6,2);
    // x
    jacobian.block(0,1,6,1)= J.block(0,0,6,1);
    // y
    jacobian.block(0,2,6,1)= J.block(0,1,6,1);
    
    jacobian.block(0,3,6,arm_dimension_)= J.block(0,14,6,arm_dimension_);
    
    return jacobian;
}

matrix_t FG_eval::get_joint_jacobian(vector_t state) {
    using ad_quaternion_t = Eigen::Quaternion<ocs2::ad_scalar_t>;
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
    using ad_vector_t = ocs2::ad_vector_t;
    using ad_scalar_t = ocs2::ad_scalar_t;
    using ad_matrix_t = ocs2::ad_matrix_t;

    matrix_t jacobian = matrix_t::Zero(6, arm_dimension_+3);
  
    const pinocchio::Model& model = pinocchioInterface_.getModel();
    // const pinocchio::Data& data = pinocchioInterface_->getData();
    // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
    pinocchio::Data data = pinocchio::Data(pinocchioInterface_.getData());
    const auto q = mappingPtr_->getPinocchioJointPosition(state);
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL;

 
    int indeeLink = model.getJointId( "SH_JOINT_6");
    
    matrix_t J = matrix_t::Zero(6, model.nv);
    

    data = pinocchio::Data(pinocchioInterface_.getData());
    pinocchio::getJointJacobian(model, data, indeeLink, rf, J);
    //theta
    jacobian.block(0,0,6,1)= J.block(0,5,6,2);
    // x
    jacobian.block(0,1,6,1)= J.block(0,0,6,1);
    // y
    jacobian.block(0,2,6,1)= J.block(0,1,6,1);
    
    jacobian.block(0,3,6,arm_dimension_)= J.block(0,14,6,arm_dimension_);
    
    return jacobian;
}

vector_t FG_eval::get_error(vector_t state) {
  
    using ad_quaternion_t = Eigen::Quaternion<ocs2::ad_scalar_t>;
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
    using ad_scalar_t = ocs2::ad_scalar_t;

    const pinocchio::Model& model = pinocchioInterface_.getModel();
    // const pinocchio::Data& data = pinocchioInterface_->getData();
    // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
    pinocchio::Data data = pinocchio::Data(pinocchioInterface_.getData());
    const auto q = mappingPtr_->getPinocchioJointPosition(state);

 
    int indeeLink = model.getFrameId( "end_effector_link");

 
    eeposition_ = (data.oMf[indeeLink].translation());
    eeOrientation_ = matrixToQuaternion(data.oMf[indeeLink].rotation());

    vector_t error(6);

    error.block(0, 0, 3, 1) = desired_position_ - eeposition_;
    error.block(3, 0, 3, 1) = quaternionDistance(eeOrientation_, desired_orientation_);
    std::cout << "ee_error: " << error.norm() << "\n";
    return error;
}

scalar_t FG_eval::get_armAngle(vector_t state) {
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
    using ad_scalar_t = ocs2::ad_scalar_t;
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    const auto q = mappingPtr_->getPinocchioJointPosition(state);
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  
    // int ArmId = model.getFrameId( "shoulder_link");
    int ArmId = model.getFrameId( "end_effector_link");
    int indBaseLink = model.getFrameId("x_arm_link");
    pinocchio::SE3Tpl<scalar_t, 0> worldToArmLBTransf = data.oMf[ArmId];
    pinocchio::SE3Tpl<scalar_t, 0> worldToBaseLBTransf = data.oMf[indBaseLink];
    pinocchio::SE3Tpl<scalar_t, 0> baseToArmLBTransf = worldToBaseLBTransf.inverse() * worldToArmLBTransf;
    
    // auto armRotMat = baseToArmLBTransf.rotation();
    // auto armAngle = -atan2(armRotMat(1, 0), armRotMat(0, 0));
    auto armPos = baseToArmLBTransf.translation();
    auto armAngle = -atan2(armPos[1], armPos[0]);
    std::cout << "arm angle:  " << armAngle*(180/M_PI) << "\n";
    return armAngle;
}

void FG_eval::get_updatedState(vector_t state,vector_t desired_position, Eigen::Quaternion<scalar_t> desired_orientation) {
    currState_ = state;
    desired_position_ = desired_position;
    desired_orientation_ = desired_orientation;
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    const auto q = mappingPtr_->getPinocchioJointPosition(state);



    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::computeJointKinematicHessians(model, data);
    std::cout << "Finished forward kinematics" << "\n";


    jacobian_.block(0, 0, 6, arm_dimension_+3) = get_jacobian(currState_);
    jacobian_.block(0, arm_dimension_+3, 6, 6) = matrix_t::Identity(6, 6);
    // std::cout << "jacobian: \n" << std::fixed << std::setprecision(4);
    // std::cout << jacobian_ << "\n";
    auto error = get_error(currState_);
    Q_ = matrix_t::Identity(arm_dimension_+9, arm_dimension_+9);
    et_ = error.block(0, 0, 2, 1).norm();
    Q_.block(0, 0, arm_dimension_+3, arm_dimension_+3) *= Y_;
   
    if (et_ != 0) {
        scalar_t scalar_temp ;
        scalar_temp = 2 / et_;

        Q_.block(0, 0, 3, 3) *= scalar_temp ;

        //  Slack component of Q
        scalar_temp = scalar_temp / et_ / et_ / et_ / et_;
        Q_.block(arm_dimension_+ 3, arm_dimension_+ 3, 6, 6) *= scalar_temp ;
    }
    C_ = vector_t::Zero(arm_dimension_+9);  
    C_[0]= K_theta_*get_armAngle(currState_);
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL ;
    pinocchio::JointIndex indeeLink = model.getJointId( "SH_JOINT_6");
    
    auto jacobian = get_joint_jacobian(currState_);
    std::cout << "jacobian: \n" << std::fixed << std::setprecision(4);
    auto hessain = getJointKinematicHessian(model, data, indeeLink, rf);
    std::vector<Eigen::Matrix<double, 3, 6>> matrices;
    for (int z=0; z<arm_dimension_; z++)
    {
        Eigen::Matrix<double, 3, 6> split;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < arm_dimension_; ++j) { // Assuming the last dimension size is 20
                split(i, j) = hessain(i, 14+j, 14+z);
            }
        }
        matrices.push_back(split);  
        vector_t temp ;
        vector_t temp2 ;
        
        Eigen::Matrix<double, 3, 3> matrix =  jacobian.block(0, 3, 3,6)*split.transpose();
        temp  = Eigen::Map<vector_t>(matrix.data(), matrix.size());
        matrix = jacobian.block(0, 3, 3,6) * jacobian.block(0, 3, 3,6).transpose();
        matrix = matrix.inverse();
        temp2 = Eigen::Map<vector_t>(matrix.data(), matrix.size());
        jacobian_m_[z] =mt_ * (temp.transpose()*temp2).norm();
    }
    for(int i =0; i<arm_dimension_-1; i++)
    {
        C_[i+3] = -jacobian_m_[i];
    }
    std::cout << "C: " << jacobian_m_.transpose() << "\n";


    desired_vector_ = K_vel_ * error;
    
    for(int i = 0; i < 6; i++) {
        // gu_[i] = 0;
        // gl_[i] = 0;
        gu_[i] = desired_vector_[i];
        gl_[i] = desired_vector_[i];
         
    }
    // Form the joint limit velocity damper
    for (int i = 0; i < arm_dimension_; i++) {
        gu_[6+i] = (joint_upperBound_(4+i)-currState_[swerve::sh_rot_state_ind +i] - ps_) / (pi_ - ps_);
        gl_[6+i] = (-joint_upperBound_(4+i)-currState_[swerve::sh_rot_state_ind +i] + ps_)/ (pi_ - ps_);
    }
    
}

vector_t FG_eval::get_solution(vector_t state,vector_t desired_position, Eigen::Quaternion<scalar_t> desired_orientation) {
    using ad_quaternion_t = Eigen::Quaternion<ad_scalar_t>;
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
    // std::cout << "====================begin_compute: ===================" << "\n";
    ros::WallTime start_time = ros::WallTime::now();
    vector_t x(arm_dimension_+9);
    for (int i = 0; i < arm_dimension_+9; i++) {
        x[i] = solution_.x[i];
    }
    // std::cout << " desired_position :"<< desired_position.transpose() << desired_orientation_.coeffs().transpose()  <<  "\n";
    // std::cout << " curr_position :" << eeposition_.transpose() << eeOrientation_.coeffs().transpose() << "\n";
    
    get_updatedState(state,desired_position,desired_orientation);
    // std::cout << " xl_ :"<< xl_.transpose() << "\n";
    // std::cout << " xu_ :"<< xu_.transpose() << "\n";
    // std::cout << " gl_ :"<< gl_.transpose() << "\n";
    // std::cout << " gu_ :"<< gu_.transpose() << "\n";
    // std::cout << " desired_vector :"<< desired_vector_.transpose() << "\n";
    ros::WallTime middle_time = ros::WallTime::now();
    
    // std::cout << "solution_status: " << solution_.status <<  "\n";
    
    vector_t ans(solution_.x.size());

    CppAD::ipopt::solve<Dvector, FG_eval>(options_, dVectorToDvector(x), dVectorToDvector(xl_), dVectorToDvector(xu_), dVectorToDvector(gl_), dVectorToDvector(gu_), *this, solution_);
    // AugmentedLagrangian_solver_noCPPAD(dVectorToDvector(x), dVectorToDvector(xl_), dVectorToDvector(xu_), dVectorToDvector(gl_), dVectorToDvector(gu_), *this, solution_,false);
    ros::WallTime end_time = ros::WallTime::now();
    // std::cout << "solution_status: " << solution_.status <<  "\n";
    
    for(int i = 0;i < solution_.x.size();i++)
    {
        ans[i]=solution_.x[i];
    }

    if (et_>0.5)
    {
        ans *= (0.7/et_);
    }
    else{
        ans *= 1.4;
    }
    // std::cout << "solution: " << ans.transpose() << "\n";
    // std::cout << "function value:" << solution_.g << "\n";
    // // std::cout <<"eqal_compute " << (jacobian_*ans - desired_vector_).transpose() << "\n";
    // std::cout << " object value:" << solution_.obj_value << "\n";
    
    double duration_ms = (middle_time - start_time).toNSec() / 1e6;
    // std::cout << "The operation took: " << duration_ms <<" ms" <<"\n";
     duration_ms = (end_time - middle_time).toNSec() / 1e6;
    // std::cout << "The operation took: " << duration_ms <<" ms" <<"\n";
    // std::cout << "====================finish_compute: ===================" << std::endl;
    return ans;
}

vector_t FG_eval::baseKinemics(const vector_t& baseInput)
{
    using vector3 = Eigen::Matrix<scalar_t, 3, 1>;
    const scalar_t wheelRadius = 0.065;
    const pinocchio::Model& model = pinocchioInterface_.getModel();
    // const pinocchio::Data& data = pinocchioInterface_->getData();
    // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
    pinocchio::Data data = pinocchio::Data(pinocchioInterface_.getData());
    // Get index
  int indBaseLink = model.getFrameId("base_link");
  //LB
  // int indBrakeLB = model.getFrameId("left_back_brake_joint");
  int indWheelLB = model.getFrameId("left_back_wheel_joint");
  int indSteerlLB = model.getFrameId("left_back_steer_joint");
  //LF
  // int indBrakeLF = model.getFrameId("left_front_brake_joint");
  int indWheelLF = model.getFrameId("left_front_wheel_joint");
  int indSteerlLF = model.getFrameId("left_front_steer_joint");
  //RB
  // int indBrakeRB = model.getFrameId("right_back_brake_joint");
  int indWheelRB = model.getFrameId("right_back_wheel_joint");
  int indSteerlRB = model.getFrameId("right_back_steer_joint");
  //RF
  // int indBrakeRF = model.getFrameId("right_front_brake_joint");
  int indWheelRF = model.getFrameId("right_front_wheel_joint");
  int indSteerlRF = model.getFrameId("right_front_steer_joint");

  // Base velocities 
  vector3 baseLinVel = Eigen::Matrix<scalar_t, 3, 1>::Zero();
  baseLinVel[0] = baseInput[1];
  baseLinVel[1] = baseInput[2];

  vector3 baseAngVel = Eigen::Matrix<scalar_t, 3, 1>::Zero();
  baseAngVel[2] = baseInput[0];

  vector3 r_WP = Eigen::Matrix<scalar_t, 3, 1>::Zero();
  r_WP[2] = wheelRadius;
  

  // Transformation matrix
  pinocchio::SE3Tpl<scalar_t, 0> worldToBaseTransf = data.oMf[indBaseLink];

  //LB
  pinocchio::SE3Tpl<scalar_t, 0> worldToWheelLBTransf = data.oMf[indWheelLB];
  pinocchio::SE3Tpl<scalar_t, 0> worldToSteerLBTransf = data.oMf[indSteerlLB];
  pinocchio::SE3Tpl<scalar_t, 0> steerLBToBase = worldToSteerLBTransf.inverse() * worldToBaseTransf;

  //LF
  pinocchio::SE3Tpl<scalar_t, 0> worldToWheelLFTransf = data.oMf[indWheelLF];
  pinocchio::SE3Tpl<scalar_t, 0> worldToSteerLFTransf = data.oMf[indSteerlLF];
  pinocchio::SE3Tpl<scalar_t, 0> steerLFToBase = worldToSteerLFTransf.inverse() * worldToBaseTransf;

  //RB
  pinocchio::SE3Tpl<scalar_t, 0> worldToWheelRBTransf = data.oMf[indWheelRB];
  pinocchio::SE3Tpl<scalar_t, 0> worldToSteerRBTransf = data.oMf[indSteerlRB];
  pinocchio::SE3Tpl<scalar_t, 0> steerRBToBase = worldToSteerRBTransf.inverse() * worldToBaseTransf;

  //RF
  pinocchio::SE3Tpl<scalar_t, 0> worldToWheelRFTransf = data.oMf[indWheelRF];
  pinocchio::SE3Tpl<scalar_t, 0> worldToSteerRFTransf = data.oMf[indSteerlRF];
  pinocchio::SE3Tpl<scalar_t, 0> steerRFToBase = worldToSteerRFTransf.inverse() * worldToBaseTransf;

  // LB LF RB RF
  vector_t wheelVel = vector_t::Zero(4);
  vector_t steerAngle = vector_t::Zero(4);
  vector_t steerVel = vector_t::Zero(4);

  vector_t temp_v = (baseLinVel + baseAngVel.cross(steerLBToBase.inverse().translation()));
  vector_t temp_v2 = (baseLinVel + baseAngVel.cross(steerLFToBase.inverse().translation()));
  vector_t temp_v3 = (baseLinVel + baseAngVel.cross(steerRBToBase.inverse().translation()));
  vector_t temp_v4 = (baseLinVel + baseAngVel.cross(steerRFToBase.inverse().translation()));


  if(baseLinVel.norm()+baseAngVel.norm()>0.02)
  {

    wheelVel[0] = (steerLBToBase.rotation()*temp_v)[0]/wheelRadius;
    wheelVel[1] = -(steerLFToBase.rotation()*temp_v2)[0]/wheelRadius;
    wheelVel[2] = -(steerRBToBase.rotation()*temp_v3)[0]/wheelRadius;
    wheelVel[3] = (steerRFToBase.rotation()*temp_v4)[0]/wheelRadius;

    steerAngle[0] = -atan2(temp_v[1],temp_v[0]);
    steerAngle[1] = -atan2(temp_v2[1],temp_v2[0]);
    steerAngle[2] = -atan2(temp_v3[1],temp_v3[0]);
    steerAngle[3] = -atan2(temp_v4[1],temp_v4[0]);
  }
  else{
    for(int i=0; i<4; i++)
    {

        steerAngle[i] = currState_(swerve::lb_steer_state_ind + 2*i);

        wheelVel[i] = 0;

    }
  }
 //print the currState_
    std::cout << "currState_: " << currState_.transpose() << "\n";
  vector_t solution = vector_t::Zero(8);
  for(int i=0; i<4; i++)
  {
    // if(fabs(currState_(swerve::lb_steer_state_ind + 2*i) - steerAngle[i])>M_PI/2)
    if(fabs(steerAngle[i])>7*M_PI/8)
    {
      steerAngle[i] = normalizeAngle(steerAngle[i] + M_PI);
    }
    steerVel[i] = (currState_(swerve::lb_steer_state_ind + 2*i) - steerAngle[i])*KpSteer_;

    if (steerVel[i] > upperBound_[swerve::lb_steer_input_ind+2*i])
    {
      steerVel[i] = upperBound_[swerve::lb_steer_input_ind+2*i];
    }
    else if (steerVel[i] < -upperBound_[swerve::lb_steer_input_ind+2*i])
    {
      steerVel[i] = -upperBound_[swerve::lb_steer_input_ind+2*i];
    }

    if (wheelVel[i] > upperBound_[swerve::lb_wheel_input_ind+2*i])
    {
      wheelVel[i] = upperBound_[swerve::lb_wheel_input_ind+2*i];
    }
    else if (wheelVel[i] < -upperBound_[swerve::lb_wheel_input_ind+2*i])
    {
      wheelVel[i] = -upperBound_[swerve::lb_wheel_input_ind+2*i];
    }
  }
  std::cout << "wheelVel: " << wheelVel.transpose() << "\n";
  std::cout << "steerAngle: " << steerAngle.transpose() << "\n";
  solution.head(4) = wheelVel;
  solution.tail(4) = -steerVel;

  return solution;
}
FG_eval::Dvector FG_eval::dVectorToDvector(const vector_t& adVec_t)
{
    Dvector Dvec(adVec_t.size());
    for (int i=0;i<adVec_t.size();i++)
    {
        Dvec[i]=adVec_t[i];
    }
    return Dvec;
}
double normalizeAngle(double angle) {
    // 将角度标准化到0到2*pi
    angle = fmod(angle, 2 * M_PI);
    // 如果angle为负，将其转换为正值
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    // 将角度调整到-pi到pi
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

bool FG_eval::AugmentedLagrangian_solver(Dvector    xi, 
                                        Dvector     xl, 
                                        Dvector     xu, 
                                        Dvector     gl, 
                                        Dvector     gu, 
                                        FG_eval&    fg_eval,
                                        CppAD::ipopt::solve_result<Dvector>& solution,
                                        bool        print)
{
   std::random_device rd;  // 
   std::mt19937 gen(rd()); // 
   std::uniform_real_distribution<> dis(-0.5, 0.5);
   size_t nx = xi.size();
   size_t ng = gl.size();
   
   ADvector fg(ng+1);
   ADvector x(nx);
   ADvector eqconstraint;
   ADvector inqconstraint;
   
   
    CppAD::Independent(x);
    fg_eval(fg,x);
    ADvector objectFunc(1);
    objectFunc[0] = fg[0];
    CppAD::ADFun<double>Fg(x,objectFunc);
    Fg.optimize();
// =========set up the constraint=========
    CppAD::Independent(x);
    fg_eval(fg,x);
    for (int i = 0; i < ng; i++)
    {
        if(gl[i] == gu[i])
        {
            eqconstraint.push_back(fg[i+1] - AD<double>(gl[i]));
        }
        else{
            if(gl[i]<-1.0e15){
                inqconstraint.push_back(fg[i+1] - AD<double>(gu[i]));
            }
            else if(gu[i]>1.0e15){
                inqconstraint.push_back(AD<double>(gl[i]) - fg[i+1]);
            }
            else{
                AD<double> temp;
                inqconstraint.push_back(fg[i+1] - AD<double>(gu[i]));
                inqconstraint.push_back(AD<double>(gl[i]) - fg[i+1]);
            }      
        } 
    }
    size_t neq = eqconstraint.size();
    size_t nInq = inqconstraint.size();
    eqconstraint.push_vector(inqconstraint);
    CppAD::ADFun<double>constraint(CppADVectorToEigenMatrix(x),CppADVectorToEigenMatrix(eqconstraint));
    constraint.optimize();
// =========set the slover parameter=========
    vector_t lamda(neq);
    vector_t u(nInq);
    
    for(int i=0; i<neq; i++)
    {
        lamda[i] = 1;
    }
    for(int i=0; i<nInq; i++)
    {
        u[i] = 1;
    }
    // penalty_factor > 1 
    double penalty_factor=2;
    double Accuracy=0.001;
    double viol=1/penalty_factor;
    
    // Augmented Lagrangian method

    int count = 0;
    double pre_viol = 0;
    double con_viol=0;
    vector_t x_value = vector_t::Zero(nx);
    for(int i=0; i<nx; i++)
    {
        x_value[i] = xi[i];
    }
    do 
    {   
        //    Gradient descent method
        vector_t gradient;
        double alpha = 0.01;
        double beta = 0.5;
        double T=0;
        double  LagrangianFunVal;
        vector_t temp;
        vector_t d_x;
        double d_value;
        int loop = 0;

        auto calculateLagrangianGradient = [&] (const vector_t& x_value) ->  vector_t{
            auto constraintVal = constraint.Forward(0, x_value);
            vector_t gradient = Fg.Jacobian(x_value);
            vector_t constraintJacobian = constraint.Jacobian(x_value);
            for (int i = 0; i < neq; i++) {
                gradient += (lamda[i] + penalty_factor*constraintVal[i])*constraintJacobian.segment(i*nx, nx);
            }
            for (int i = 0; i < nInq; i++) {
                gradient += (u[i] + penalty_factor*std::max(-u[i]/penalty_factor, constraintVal[neq+i]))
                            *constraintJacobian.segment((neq+i)*nx, nx);
            }

            return gradient;
        };

        auto calculateLagrangian = [&] (const vector_t& x_value) ->  double{
            double LagrangianFunVal = Fg.Forward(0, x_value)[0]; 
            auto constraintVal = constraint.Forward(0, x_value);
            for (int i = 0; i < neq; i++) {
                LagrangianFunVal += lamda[i]*constraintVal[i] + 0.5*penalty_factor*constraintVal[i]*constraintVal[i];
            }
            for (int i = 0; i < nInq; i++) {
                double adjustedConstraint = std::max(u[i]/penalty_factor + constraintVal[neq+i], 0.0);
                LagrangianFunVal += 0.5*penalty_factor*(adjustedConstraint*adjustedConstraint - u[i]*u[i]/penalty_factor/penalty_factor);
            }
            return LagrangianFunVal;
        };


        //solve unconstraint optimization problem
        do{
            double LagrangianFunVal = calculateLagrangian(x_value); 

            gradient = calculateLagrangianGradient(x_value);
            for(int i=0; i<nx; i++)
            {
                if(x_value[i] == xl[i] && gradient[i] > 0)
                {
                    gradient[i] = 0;
                }
                if (x_value[i] == xu[i] && gradient[i] < 0)
                {
                    gradient[i] = 0;
                }  
            }
            //d_x = -Hessain.inverse()*gradient;
            d_x = -gradient;
            // Backtracking line search
            double t = 1;
            do{
                t = beta*t;
                temp =x_value + t*d_x;
                for(int i=0; i<nx; i++)
                {
                    temp[i] = std::max(xl[i],std::min(xu[i],temp[i]));
                }

                T = t;
            }while (calculateLagrangian(temp)> LagrangianFunVal + alpha*t*gradient.transpose()*d_x );
            x_value = x_value + t*d_x;
            for(int i=0; i<nx; i++)
            {
                x_value[i] = std::max(xl[i],std::min(xu[i],x_value[i]));
            }
            
            loop++;
            if (loop > 2000)
            {
                // std::cerr<<"my solver: loop "<<loop<<"d_value:  "<<d_value<<"" <<x_value.transpose()<<" T:"<<T<<std::endl;
                break;
            }
        }while(gradient.norm()>=Accuracy );
        // constraint_violation
        vector_t constraintVal = constraint.Forward(0,x_value);
        for (int i=0; i<nInq; i++)
        {
            constraintVal[neq+i]=std::max( constraintVal[neq+i],-u[i]/penalty_factor);
        }
        con_viol = constraintVal.norm();

        if((gradient.norm() <= 0.01 && con_viol <= 0.001) || count > 100)
        {
            solution_.x = EigenMatrixToCppADVector(x_value);
            solution_.obj_value = Fg.Forward(0,x_value)[0];
            solution_.status = CppAD::ipopt::solve_result<Dvector>::status_type::success;
            vector_t functionValue = Fg.Forward(0,x_value).segment(1,ng);
            solution_.g = EigenMatrixToCppADVector(functionValue);

            return true;
        }
        else if(con_viol<=viol)
        {
            lamda = lamda + penalty_factor*constraintVal.segment(0,neq);
            for(int i=0; i<nInq; i++)
            {
                u[i] = std::max(0.0, u[i]+penalty_factor*constraintVal[neq+i]);
            }
            viol = viol/2;
            // Accuracy=Accuracy/2;
        }
        else
        {
            penalty_factor = 2*penalty_factor;
            // Accuracy=1/CppAD::Value(penalty_factor);
            viol=1/sqrt(sqrt(penalty_factor));
        } 
        if(print)
        {
            std::cerr<<"my solver: penalty_factor "<<penalty_factor<<std::endl;
            std::cerr<<"my solver: Accuracy "<<Accuracy<<std::endl;
            std::cerr<<"my solver: con_viol "<<con_viol<<std::endl;
            std::cerr<<"my solver: viol "<<viol<<std::endl;
            std::cerr<<"my solver: solution "<<x_value.transpose()<<std::endl;
            std::cerr<<"my solver: gradient "<<gradient.transpose()<<std::endl;
            std::cerr<<"my solver: Lamda "<<lamda[0]<<std::endl;
           
            std::cerr<<"my solver: object value"<<Fg.Forward(0,x_value)<<std::endl;
            std::cerr<<"my solver: count "<<count<<std::endl;
        
            std::cerr<<"============================="<<std::endl;
        }
        count++;
   }while (count < 100);
    solution_.status = CppAD::ipopt::solve_result<Dvector>::status_type::maxiter_exceeded;
    return false;
}

bool FG_eval::AugmentedLagrangian_solver_noCPPAD(Dvector    xi, 
                                                Dvector     xl, 
                                                Dvector     xu, 
                                                Dvector     gl, 
                                                Dvector     gu, 
                                                FG_eval&    fg_eval,
                                                CppAD::ipopt::solve_result<Dvector>& solution,
                                                bool        print)
{
    size_t nx = xi.size();
    size_t ng = gl.size();
    vector_t lamda(nx);
    vector_t u(2*nx);

    for(int i=0; i<nx; i++)
    {
        lamda[i] = 1;
    }
    for(int i=0; i<2*nx; i++)
    {
        u[i] = 1;
    }
    // penalty_factor > 1 
    double penalty_factor=2;
    double Accuracy=0.001;
    double viol=1/penalty_factor;
    int count = 0;
    double pre_viol = 0;
    double con_viol=0;
    vector_t x_value = vector_t::Zero(nx);
    for(int i=0; i<nx; i++)
    {
        x_value[i] = xi[i];
    }

    auto objectValue = [&] (const vector_t& x_value) ->  double{
        double objectValue ;
        objectValue = (0.5*(x_value.transpose()*Q_*x_value) + C_.transpose()*x_value)(0,0);
        return objectValue;
    };

    auto calculateLagrangian = [&] (const vector_t& x_value) ->  double{
        double LagrangianFunVal = objectValue(x_value); 
        // LagrangianFunVal += lamda[i]*constraintVal[i] + 0.5*penalty_factor*constraintVal[i]*constraintVal[i];
        vector_t equalconstraintVal = jacobian_*x_value - desired_vector_;
        LagrangianFunVal += (lamda.transpose()*equalconstraintVal)(0,0) + 0.5*(penalty_factor*equalconstraintVal.transpose()*equalconstraintVal)(0,0);
        
        vector_t inequalconstraintVal(12);
        for(int i=0; i<6; i++)
        {
            inequalconstraintVal[2*i] = x_value[i+3]-gu[6+i];
            inequalconstraintVal[2*i+1] = gl[6+i]-x_value[i+3];
        }

        for(int i=0; i<12; i++)
        {
            double adjustedConstraint = std::max(u[i]/penalty_factor + inequalconstraintVal[i], 0.0);
            LagrangianFunVal += 0.5*penalty_factor*(adjustedConstraint*adjustedConstraint - u[i]*u[i]/penalty_factor/penalty_factor);
        }
        return LagrangianFunVal;
    };

    auto calculateLagrangianGradient = [&] (const vector_t& x_value) ->  vector_t{
        vector_t gradient = Q_*x_value + C_ + jacobian_.transpose()*lamda + penalty_factor*jacobian_.transpose()*(jacobian_*x_value - desired_vector_);
        for (int i = 0; i < 6; i++) {
           if(u[2*i]/penalty_factor + x_value[i+3]-gu[6+i] > 0)
           {
               gradient[i+3] += penalty_factor*(u[i]/penalty_factor + x_value[i+3]-gu[6+i]);
           }
           else{
               gradient[i+3] += 0;
           }

           if(u[2*i+1]/penalty_factor + gl[6+i]-x_value[i+3] > 0)
           {
               gradient[i+3] -= penalty_factor*(u[2*i+1]/penalty_factor + gl[6+i]-x_value[i+3]);
           }
           else{
               gradient[i+3] += 0;
           }
        }

        return gradient;
    };

    do 
    {   
        //    Gradient descent method
        vector_t gradient;
        double alpha = 0.2;
        double beta = 0.5;
        double T=0;
        double  LagrangianFunVal;
        vector_t temp;
        vector_t d_x;
        double d_value;
        int loop = 0;

        //solve unconstraint optimization problem
        do{
            double LagrangianFunVal = calculateLagrangian(x_value); 
            gradient = calculateLagrangianGradient(x_value);
            // std::cout << "gradient: " << gradient.transpose() << "\n";
            // std::cout << "gradient_cal: " << calculateLagrangianGradient(x_value).transpose() << "\n";
            // std::cout << "gradient_2: " << calculateLagrangianGradientcppad2(x_value).transpose() << "\n";
            // std::cout << "gradient_cal2: " << calculateLagrangianGradient2(x_value).transpose() << "\n";
            
            // std::cout << "======================================" << x_value.transpose() << "\n";    
            for(int i=0; i<nx; i++)
            {
                if(x_value[i] == xl[i] && gradient[i] > 0)
                {
                    gradient[i] = 0;
                }
                if (x_value[i] == xu[i] && gradient[i] < 0)
                {
                    gradient[i] = 0;
                }  
            }
            //d_x = -Hessain.inverse()*gradient;
            d_x = -gradient;
            // Backtracking line search
            double t = 0.01;
            do{
                t = beta*t;
                temp =x_value + t*d_x;
                for(int i=0; i<nx; i++)
                {
                    temp[i] = std::max(xl[i],std::min(xu[i],temp[i]));
                }

                T = t;
            }while (calculateLagrangian(temp)> LagrangianFunVal + alpha*t*gradient.transpose()*d_x );
            x_value = x_value + t*d_x;
            for(int i=0; i<nx; i++)
            {
                x_value[i] = std::max(xl[i],std::min(xu[i],x_value[i]));
            }
            
            loop++;
            if (loop > 2000)
            {
                // std::cerr<<"my solver: loop "<<loop<<"d_value:  "<<"" <<"x_value: "<<x_value.transpose()<<" T:"<<T<<std::endl;
                break;
            }
        }while(gradient.norm()>=Accuracy );
        // constraint_violation
        vector_t equalconstraintVal = jacobian_*x_value - desired_vector_;
        vector_t inequalconstraintVal(12);
        for(int i=0; i<6; i++)
        {
            inequalconstraintVal[i] = x_value[i+3]-gu[6+i];
            inequalconstraintVal[i+6] = gl[6+i]-x_value[i+3];
            if(inequalconstraintVal[i] < 0)
            {
                inequalconstraintVal[i] = 0;
            }
        }
        for (int i=0; i<12; i++)
        {
            inequalconstraintVal[i]=std::max( inequalconstraintVal[i],-u[i]/penalty_factor);
        }

        vector_t constraintVal(18);
        constraintVal.segment(0,6) = equalconstraintVal;
        constraintVal.segment(6,12) = inequalconstraintVal;
        con_viol = constraintVal.norm();

        if((gradient.norm() <= 0.01 && con_viol <= 0.001) || count > 100)
        {
            solution_.x = EigenMatrixToCppADVector(x_value);
            solution_.obj_value = objectValue(x_value);
            solution_.status = CppAD::ipopt::solve_result<Dvector>::status_type::success;

            return true;
        }
        else if(con_viol<=viol)
        {
            lamda = lamda + penalty_factor*constraintVal.segment(0,6);
            for(int i=0; i<12; i++)
            {
                u[i] = std::max(0.0, u[i]+penalty_factor*constraintVal[6+i]);
            }
            viol = viol/2;
            // Accuracy=Accuracy/2;
        }
        else
        {
            penalty_factor = 2*penalty_factor;
            // Accuracy=1/CppAD::Value(penalty_factor);
            viol=1/sqrt(sqrt(penalty_factor));
        } 
        if(print)
        {
            std::cerr<<"my solver: penalty_factor "<<penalty_factor<<std::endl;
            std::cerr<<"my solver: Accuracy "<<Accuracy<<std::endl;
            std::cerr<<"my solver: con_viol "<<con_viol<<std::endl;
            std::cerr<<"my solver: viol "<<viol<<std::endl;
            std::cerr<<"my solver: solution "<<x_value.transpose()<<std::endl;
            std::cerr<<"my solver: gradient "<<gradient.transpose()<<std::endl;
            std::cerr<<"my solver: Lamda "<<lamda[0]<<std::endl;
           
            std::cerr<<"my solver: object value"<<objectValue(x_value)<<std::endl;
            std::cerr<<"my solver: count "<<count<<std::endl;
     
            std::cerr<<"============================="<<std::endl;
        }
           count++;
   }while (count < 100);
    solution_.status = CppAD::ipopt::solve_result<Dvector>::status_type::maxiter_exceeded;
    return false;

}


void FG_eval::Set_currentState(vector_t state)
{
    currState_ = state;
}
} // namespace swerve_qp_ros_control
} // namespace ocs2
