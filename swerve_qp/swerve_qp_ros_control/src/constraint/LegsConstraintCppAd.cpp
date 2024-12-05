#include "constraint/LegsConstraintCppAd.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2 {
namespace swerve {

LegsConstraintCppAd::LegsConstraintCppAd(const std::string& prefix, SwerveModelInfo swerveModelInfo, const std::string& libraryFolder, bool recompileLibraries,
                                         const SwerveReferenceManager& referenceManager)
    : StateConstraintCppAd(ConstraintOrder::Linear), referenceManagerPtr_(&referenceManager), swerveModelInfo_(swerveModelInfo) {
  initialize(swerveModelInfo_.stateDim, 4, prefix, libraryFolder, recompileLibraries, true);
}

LegsConstraintCppAd::LegsConstraintCppAd(const LegsConstraintCppAd& rhs)
    : StateConstraintCppAd(rhs), referenceManagerPtr_(rhs.referenceManagerPtr_) {}

LegsConstraintCppAd* LegsConstraintCppAd::clone() const {
  return new LegsConstraintCppAd(*this);
}

ad_vector_t LegsConstraintCppAd::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  ad_vector_t result = Eigen::Matrix<ad_scalar_t, 4, 1>::Zero();
  // result << state(lb_brake_state_ind) - parameters(0), state(lf_brake_state_ind) - parameters(1), state(rb_brake_state_ind) - parameters(2),
  //     state(rf_brake_state_ind) - parameters(3);

  return result;
}

vector_t LegsConstraintCppAd::getParameters(scalar_t time) const {
  vector_t parameters = Eigen::Matrix<scalar_t, 4, 1>::Zero();
  parameters << referenceManagerPtr_->getTargetTrajectories().getDesiredState(time).tail(4);
  return parameters;
}

}  // namespace swerve
}  // namespace ocs2