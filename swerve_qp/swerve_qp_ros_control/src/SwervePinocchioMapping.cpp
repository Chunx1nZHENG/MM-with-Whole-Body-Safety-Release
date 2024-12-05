#include "SwervePinocchioMapping.hpp"

namespace ocs2 {
namespace swerve {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
SwervePinocchioMappingTpl<SCALAR>::SwervePinocchioMappingTpl(SwerveModelInfo swerveModelInfo)
    : swerveModelInfo_(std::move(swerveModelInfo)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
SwervePinocchioMappingTpl<SCALAR>* SwervePinocchioMappingTpl<SCALAR>::clone() const {
  return new SwervePinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SwervePinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
 
  vector_t states(swerveModelInfo_.stateDim + 4);
  switch (swerveModelInfo_.armPresent) {
    case true: {
      states << state(x_state_ind), state(y_state_ind), state(z_state_ind), state(x_quat_state_ind), state(y_quat_state_ind),
          state(z_quat_state_ind), state(w_quat_state_ind), state(lb_steer_state_ind),
          sin(state(lb_wheel_state_ind)), -cos(state(lb_wheel_state_ind)),  state(lf_steer_state_ind),
          sin(state(lf_wheel_state_ind)), -cos(state(lf_wheel_state_ind)),  state(rb_steer_state_ind),
          sin(state(rb_wheel_state_ind)), -cos(state(rb_wheel_state_ind)),  state(rf_steer_state_ind),
          sin(state(rf_wheel_state_ind)), -cos(state(rf_wheel_state_ind)), state.tail(swerveModelInfo_.armDim);
      break;
    }
    case false: {
      states << state(x_state_ind), state(y_state_ind), state(z_state_ind), state(x_quat_state_ind), state(y_quat_state_ind),
          state(z_quat_state_ind), state(w_quat_state_ind),  state(lb_steer_state_ind),
          sin(state(lb_wheel_state_ind)), -cos(state(lb_wheel_state_ind)),  state(lf_steer_state_ind),
          sin(state(lf_wheel_state_ind)), -cos(state(lf_wheel_state_ind)), state(rb_steer_state_ind),
          sin(state(rb_wheel_state_ind)), -cos(state(rb_wheel_state_ind)), state(rf_steer_state_ind),
          sin(state(rf_wheel_state_ind)), -cos(state(rf_wheel_state_ind));
      break;
    }
  }

  return states;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SwervePinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  vector_t vPinocchio = vector_t::Zero(swerveModelInfo_.pinnochioInputDim);
  using vector_3t = Eigen::Matrix<SCALAR, 3, 1>;
  using quat_t = Eigen::Quaternion<SCALAR>;

  vector_3t currentPosition = state.template head<3>();
  quat_t currentOrientation(state.template segment<4>(3));

  vector_3t linVelBaseFrame = vector_3t::Zero();
  linVelBaseFrame[0] = input[x_input_ind];
  linVelBaseFrame[1] = input[y_input_ind];

  vector_3t angularVelBaseFrame = vector_3t::Zero();
  angularVelBaseFrame[2] = input[theta_input_ind];

  vPinocchio.template head<3>() = currentOrientation * linVelBaseFrame;
  vPinocchio.template segment<3>(3) = currentOrientation * angularVelBaseFrame;

  vPinocchio.template tail(vPinocchio.template size() - 6) = input.template tail(input.size() - 3);
  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SwervePinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  throw std::logic_error("Not implemented yet!");
  return {Jq, Jv};
}

// explicit template instantiation
template class ocs2::swerve::SwervePinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::swerve::SwervePinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace swerve
}  // namespace ocs2