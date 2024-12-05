#include "cost/NominalArmStateCppAd.hpp"
namespace ocs2 {
namespace swerve {

NominalArmStateCppAd::NominalArmStateCppAd(const std::string& prefix, SwerveModelInfo swerveModelInfo,
                                           const SwerveReferenceManager& referenceManager, const std::string& libraryFolder,
                                           bool recompileLibraries, int index)
    : StateConstraintCppAd(ConstraintOrder::Linear),
      swerveModelInfo_(swerveModelInfo),
      referenceManagerPtr_(&referenceManager),
      index_(index) {
  initialize(swerveModelInfo_.stateDim, 0, prefix, libraryFolder, recompileLibraries, true);
}

NominalArmStateCppAd::NominalArmStateCppAd(const NominalArmStateCppAd& rhs)
    : StateConstraintCppAd(rhs), referenceManagerPtr_(rhs.referenceManagerPtr_), index_(rhs.index_) {}

NominalArmStateCppAd* NominalArmStateCppAd::clone() const {
  return new NominalArmStateCppAd(*this);
}

bool NominalArmStateCppAd::isActive(scalar_t time) const {
  return true;
}

ad_vector_t NominalArmStateCppAd::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  ad_vector_t result = Eigen::Matrix<ad_scalar_t, 1, 1>::Zero();

  result << state(modelSettings_.armJointsStateIndex[index_]);

  return result;
}

}  // namespace swerve
}  // namespace ocs2