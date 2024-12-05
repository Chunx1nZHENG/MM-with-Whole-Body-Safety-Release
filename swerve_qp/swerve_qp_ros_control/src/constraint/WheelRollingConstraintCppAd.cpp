#include "constraint/WheelRollingConstraintCppAd.hpp"
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/fwd.hpp>
#include "pinocchio/multibody/fcl.hpp"
namespace ocs2 {
namespace swerve {

WheelRollingConstraintCppAd::WheelRollingConstraintCppAd(const PinocchioInterface& pinocchioInterface,
                                                         const PinocchioStateInputMapping<ad_scalar_t>& mapping, const std::string& prefix,
                                                         const std::string& libraryFolder, bool recompileLibraries, SwerveModelInfo swerveModelInfo)
    : StateInputConstraintCppAd(ConstraintOrder::Linear), mappingPtr_(mapping.clone()), swerveModelInfo_(swerveModelInfo) {
  pinocchioInterfacePtr_ = &pinocchioInterface;
  mappingPtr_->setPinocchioInterface(pinocchioInterfacePtr_->toCppAd());
  initialize(swerveModelInfo_.stateDim, swerveModelInfo_.inputDim, 0, prefix, libraryFolder, recompileLibraries, true);
}

WheelRollingConstraintCppAd::WheelRollingConstraintCppAd(const WheelRollingConstraintCppAd& rhs)
    : StateInputConstraintCppAd(rhs), pinocchioInterfacePtr_(rhs.pinocchioInterfacePtr_), mappingPtr_(rhs.mappingPtr_->clone()), swerveModelInfo_(rhs.swerveModelInfo_) {}


ad_vector_t WheelRollingConstraintCppAd::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                            const ad_vector_t& parameters) const {
  ad_vector_t constraint = Eigen::Matrix<ad_scalar_t, 8, 1>::Zero();
  ad_scalar_t wheelRadius;
  wheelRadius = 0.065;

  auto PinocchioInterfaceCppAd = pinocchioInterfacePtr_->toCppAd();
  const auto& model = PinocchioInterfaceCppAd.getModel();
  auto& data = PinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mappingPtr_->getPinocchioJointPosition(state);

  // Compute forward kinematics
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  

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
  ad_vector3 baseLinVel = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  baseLinVel[0] = input[x_input_ind];
  baseLinVel[1] = input[y_input_ind];

  ad_vector3 baseAngVel = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  baseAngVel[2] = input[theta_input_ind];

  ad_vector3 normalVector = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  normalVector[2] = 1;

  // Transformation matrix
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToBaseTransf = data.oMf[indBaseLink];

  //LB
  // pinocchio::SE3Tpl<ad_scalar_t, 0> worldToBrakeLBTransf = data.oMf[indBrakeLB];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToWheelLBTransf = data.oMf[indWheelLB];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToSteerLBTransf = data.oMf[indSteerlLB];

  // pinocchio::SE3Tpl<ad_scalar_t, 0> brakeLBToBase = worldToBrakeLBTransf.inverse() * worldToBaseTransf;
  // pinocchio::SE3Tpl<ad_scalar_t, 0> steerLBToBrakeLB = worldToSteerLBTransf.inverse() * worldToBrakeLBTransf;
  pinocchio::SE3Tpl<ad_scalar_t, 0> steerLBToBase = worldToSteerLBTransf.inverse() * worldToBaseTransf;

  //LF
  // pinocchio::SE3Tpl<ad_scalar_t, 0> worldToBrakeLFTransf = data.oMf[indBrakeLF];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToWheelLFTransf = data.oMf[indWheelLF];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToSteerLFTransf = data.oMf[indSteerlLF];

  // pinocchio::SE3Tpl<ad_scalar_t, 0> brakeLFToBase = worldToBrakeLFTransf.inverse() * worldToBaseTransf;
  // pinocchio::SE3Tpl<ad_scalar_t, 0> steerLFToBrakeLF = worldToSteerLFTransf.inverse() * worldToBrakeLFTransf;
  pinocchio::SE3Tpl<ad_scalar_t, 0> steerLFToBase = worldToSteerLFTransf.inverse() * worldToBaseTransf;

  //RB
  // pinocchio::SE3Tpl<ad_scalar_t, 0> worldToBrakeRBTransf = data.oMf[indBrakeRB];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToWheelRBTransf = data.oMf[indWheelRB];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToSteerRBTransf = data.oMf[indSteerlRB];

  // pinocchio::SE3Tpl<ad_scalar_t, 0> brakeRBToBase = worldToBrakeRBTransf.inverse() * worldToBaseTransf;
  // pinocchio::SE3Tpl<ad_scalar_t, 0> steerRBToBrakeRB = worldToSteerRBTransf.inverse() * worldToBrakeRBTransf;
  pinocchio::SE3Tpl<ad_scalar_t, 0> steerRBToBase = worldToSteerRBTransf.inverse() * worldToBaseTransf;

  //RF
  // pinocchio::SE3Tpl<ad_scalar_t, 0> worldToBrakeRFTransf = data.oMf[indBrakeRF];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToWheelRFTransf = data.oMf[indWheelRF];
  pinocchio::SE3Tpl<ad_scalar_t, 0> worldToSteerRFTransf = data.oMf[indSteerlRF];

  // pinocchio::SE3Tpl<ad_scalar_t, 0> brakeRFToBase = worldToBrakeRFTransf.inverse() * worldToBaseTransf;
  // pinocchio::SE3Tpl<ad_scalar_t, 0> steerRFToBrakeRF = worldToSteerRFTransf.inverse() * worldToBrakeRFTransf;
  pinocchio::SE3Tpl<ad_scalar_t, 0> steerRFToBase = worldToSteerRFTransf.inverse() * worldToBaseTransf;

  //Distance from wheel frame to point P
  ad_vector3 r_WP = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  r_WP[2] = wheelRadius;

  // Angular velocities legs
  //LB
  // ad_vector3 ang_L_LB = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  // ang_L_LB[2] = input[lb_brake_input_ind];
  // //LF
  // ad_vector3 ang_L_LF = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  // ang_L_LF[2] = input[lf_brake_input_ind];
  // //RB
  // ad_vector3 ang_L_RB = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  // ang_L_RB[2] = input[rb_brake_input_ind];
  // //RF
  // ad_vector3 ang_L_RF = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  // ang_L_RF[2] = input[rf_brake_input_ind];

  //Angular velocities wheel
  //LB
  ad_vector3 ang_W_LB = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  ang_W_LB[1] = -input[lb_wheel_input_ind];

  //LF
  ad_vector3 ang_W_LF = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  ang_W_LF[1] = input[lf_wheel_input_ind];

  //RB
  ad_vector3 ang_W_RB = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  ang_W_RB[1] = input[rb_wheel_input_ind];

  //RF
  ad_vector3 ang_W_RF = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  ang_W_RF[1] = -input[rf_wheel_input_ind];
  
  // ad_vector3 constraintLB = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerLBToBase.inverse().translation()) + brakeLBToBase.inverse().rotation() * ang_L_LB.cross(steerLBToBrakeLB.inverse().translation()) + steerLBToBase.rotation().transpose() * ang_W_LB.cross(r_WP));
  // ad_vector3 constraintLF = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerLFToBase.inverse().translation()) + brakeLFToBase.inverse().rotation() * ang_L_LF.cross(steerLFToBrakeLF.inverse().translation()) + steerLFToBase.rotation().transpose() * ang_W_LF.cross(r_WP));
  // ad_vector3 constraintRB = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerRBToBase.inverse().translation()) + brakeRBToBase.inverse().rotation() * ang_L_RB.cross(steerRBToBrakeRB.inverse().translation()) + steerRBToBase.rotation().transpose() * ang_W_RB.cross(r_WP));
  // ad_vector3 constraintRF = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerRFToBase.inverse().translation()) + brakeRFToBase.inverse().rotation() * ang_L_RF.cross(steerRFToBrakeRF.inverse().translation()) + steerRFToBase.rotation().transpose() * ang_W_RF.cross(r_WP));

  ad_vector3 constraintLB = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerLBToBase.inverse().translation()) + steerLBToBase.rotation().transpose() * ang_W_LB.cross(r_WP));
  ad_vector3 constraintLF = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerLFToBase.inverse().translation()) + steerLFToBase.rotation().transpose() * ang_W_LF.cross(r_WP));
  ad_vector3 constraintRB = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerRBToBase.inverse().translation()) + steerRBToBase.rotation().transpose() * ang_W_RB.cross(r_WP));
  ad_vector3 constraintRF = worldToBaseTransf.rotation() * (baseLinVel + baseAngVel.cross(steerRFToBase.inverse().translation()) + steerRFToBase.rotation().transpose() * ang_W_RF.cross(r_WP));

  constraint <<  constraintLB.head(2), constraintLF.head(2), constraintRB.head(2), constraintRF.head(2);

  return constraint;
}

WheelRollingConstraintCppAd* WheelRollingConstraintCppAd::clone() const {
  return new WheelRollingConstraintCppAd(*this);
}

}  // namespace swerve
}  // namespace ocs2
