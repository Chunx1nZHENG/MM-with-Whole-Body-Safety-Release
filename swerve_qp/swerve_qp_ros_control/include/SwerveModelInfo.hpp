#pragma once

// C/C++
#include <string>
#include <vector>

namespace ocs2 {
namespace swerve {

enum InputIndex {
  x_input_ind,
  y_input_ind,
  theta_input_ind,
  lb_steer_input_ind,
  lb_wheel_input_ind,
  lf_steer_input_ind,
  lf_wheel_input_ind,
  rb_steer_input_ind,
  rb_wheel_input_ind,
  rf_steer_input_ind,
  rf_wheel_input_ind,
  sh_rot_input_ind,
  sh_fle_input_ind,
  el_fle_input_ind,
  el_rot_input_ind,
  wr_fle_input_ind,
  wr_rot_input_ind,
};

enum StateIndex {
  x_state_ind,
  y_state_ind,
  z_state_ind,
  x_quat_state_ind,
  y_quat_state_ind,
  z_quat_state_ind,
  w_quat_state_ind,
  lb_steer_state_ind,
  lb_wheel_state_ind,
  lf_steer_state_ind,
  lf_wheel_state_ind,
  rb_steer_state_ind,
  rb_wheel_state_ind,
  rf_steer_state_ind,
  rf_wheel_state_ind,
  sh_rot_state_ind,
  sh_fle_state_ind,
  el_fle_state_ind,
  el_rot_state_ind,
  wr_fle_state_ind,
  wr_rot_state_ind,
};

struct ArmLimits {
  double armLowerLimits[6] = {-5.8, -1.9, -2.2, -5.8, -1.7, -3.0};
  double armUpperLimits[6] = {5.8, 1.9, 2.2, 5.8, 1.7, 3.0};
};

struct ModelSettings {
  int steersStateIndex[4] = {lb_steer_state_ind, lf_steer_state_ind, rb_steer_state_ind, rf_steer_state_ind};
  int steersInputIndex[4] = {lb_steer_input_ind, lf_steer_input_ind, rb_steer_input_ind, rf_steer_input_ind};
  int wheelsStateIndex[4] = {lb_wheel_state_ind, lf_wheel_state_ind, rb_wheel_state_ind, rf_wheel_state_ind};
  int wheelsInputIndex[4] = {lb_wheel_input_ind, lf_wheel_input_ind, rb_wheel_input_ind, rf_wheel_input_ind};
  int armJointsStateIndex[6] = {sh_rot_state_ind, sh_fle_state_ind, el_fle_state_ind, el_rot_state_ind, wr_fle_state_ind, wr_rot_state_ind};
  int armJointsInputIndex[6] = {sh_rot_input_ind, sh_fle_input_ind, el_fle_input_ind, el_rot_input_ind, wr_fle_input_ind, wr_rot_input_ind};

  ArmLimits armLimits;
};

/**
 * @brief A data structure to store manipulator information.
 *
 * The attributes are filled by resolving the URDF model parsed.
 */
struct SwerveModelInfo {
  bool armPresent;  // type of manipulator: floating-base, wheel-base, default
  size_t stateDim;  // number of states needed to define the system flow map
  size_t inputDim;  // number of inputs needed to define the system flow map
  size_t pinnochioInputDim;
  size_t armDim;  // number of DOFs in the robot arm
  size_t wheelDim;
  size_t steerDim;
  size_t brakeDim;
  size_t legsDim;
  size_t jointLimitsDim;
  std::string eeFrame;  // name of the end-effector frame of the robot

  SwerveModelInfo()
      : armPresent(false),
        stateDim(0),
        inputDim(0),
        armDim(6),
        wheelDim(4),
        steerDim(4),
        brakeDim(4),
        legsDim(4),
        jointLimitsDim(0),
        eeFrame("base_link") {}
};

}  // namespace swerve
}  // namespace ocs2