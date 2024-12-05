#include <constraint/JointLimitsConstraint.hpp>

namespace ocs2 {
namespace swerve {

vector_t JointLimitsConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  vector_t jointStates(swerveModelInfo_.jointLimitsDim);
  if (swerveModelInfo_.armPresent) {
    jointStates << state(lb_steer_state_ind), state(lf_steer_state_ind), state(rb_steer_state_ind), state(rf_steer_state_ind),
      state(sh_rot_state_ind), state(sh_fle_state_ind), state(el_fle_state_ind), state(el_rot_state_ind), state(wr_fle_state_ind),
      state(wr_rot_state_ind);
  } else {
    jointStates << state(lb_steer_state_ind), state(lf_steer_state_ind), state(rb_steer_state_ind), state(rf_steer_state_ind);
  }

  return jointStates;
}

VectorFunctionLinearApproximation JointLimitsConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  VectorFunctionLinearApproximation limits(swerveModelInfo_.jointLimitsDim, state.rows(), 0);
  limits.f << getValue(time, state, preComputation);
  limits.dfdx.setZero();
  return limits;
}

}  // namespace swerve
}  // namespace ocs2
