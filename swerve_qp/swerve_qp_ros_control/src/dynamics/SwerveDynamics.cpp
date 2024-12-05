#include <SwerveDynamics.hpp>

namespace ocs2 {
namespace swerve {

SwerveDynamics::SwerveDynamics(SwerveModelInfo swerveModelInfo, const std::string& modelName,
                               const std::string& modelFolder /*= "/tmp/ocs2"*/, bool recompileLibraries /*= true*/,
                               bool verbose /*= true*/)
    : SystemDynamicsBaseAD(), swerveModelInfo_(swerveModelInfo) {
  Base::initialize(swerveModelInfo_.stateDim, swerveModelInfo_.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

ad_vector_t SwerveDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                          const ad_vector_t& parameters) const {
  size_t stateDim = state.size();
  size_t inputDim = input.size();

  ad_vector_t velocity(stateDim);

  using ad_quat_t = Eigen::Quaternion<ad_scalar_t>;
  ad_quat_t currentOrientation(state.segment<4>(x_quat_state_ind));
  ad_quat_t deltaOrientation;
  deltaOrientation.x() = 0.0;
  deltaOrientation.y() = 0.0;
  deltaOrientation.z() = input(theta_input_ind) / 2;
  deltaOrientation.w() = 0.0;
  velocity.segment<4>(x_quat_state_ind) = (currentOrientation * deltaOrientation).coeffs();

  ad_vector_t linearInputVelocity(3);
  linearInputVelocity[0] = input[x_input_ind];
  linearInputVelocity[1] = input[y_input_ind];
  linearInputVelocity[2] = 0.0;

  velocity.segment<3>(x_state_ind) = currentOrientation * linearInputVelocity;

  // every part of the state except the base pose can just be computed by integrating the inputs
  velocity.tail(stateDim - 7) = input.tail(inputDim - 3);
  return velocity;
}

}  // namespace swerve
}  // namespace ocs2
