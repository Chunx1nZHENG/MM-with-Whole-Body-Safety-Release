#include "cost/NominalSteerStateCppAd.hpp"
namespace ocs2 {
namespace swerve {

NominalSteerStateCppAd::NominalSteerStateCppAd(const std::string& prefix, SwerveModelInfo swerveModelInfo,
                                               const SwerveReferenceManager& referenceManager, const std::string& libraryFolder,
                                               bool recompileLibraries)
    : StateConstraintCppAd(ConstraintOrder::Linear), swerveModelInfo_(swerveModelInfo), referenceManagerPtr_(&referenceManager) {
  initialize(swerveModelInfo_.stateDim, swerveModelInfo_.brakeDim, prefix, libraryFolder, recompileLibraries, true);
}

NominalSteerStateCppAd::NominalSteerStateCppAd(const NominalSteerStateCppAd& rhs)
    : StateConstraintCppAd(rhs), swerveModelInfo_(rhs.swerveModelInfo_), referenceManagerPtr_(rhs.referenceManagerPtr_) {}

NominalSteerStateCppAd* NominalSteerStateCppAd::clone() const {
  return new NominalSteerStateCppAd(*this);
}

size_t NominalSteerStateCppAd::getNumConstraints(scalar_t time) const {
  return swerveModelInfo_.brakeDim;
}

/** Get the parameter vector */
vector_t NominalSteerStateCppAd::getParameters(scalar_t time) const {
  return referenceManagerPtr_->getNominalSteerActive();
};

ad_vector_t NominalSteerStateCppAd::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  ad_vector_t result = Eigen::Matrix<ad_scalar_t, -1, 1>::Zero(swerveModelInfo_.brakeDim);
  for (size_t i = 0; i < swerveModelInfo_.brakeDim; ++i) {
    // check if parameter is less than 0.5. It should either be 0.0 when the constraint is not active or 1.0 when the constraint is active
    result[i] = CppAD::CondExpLt(parameters[i], ad_scalar_t(0.5), ad_scalar_t(0.0), state(modelSettings_.steersStateIndex[i]));
  }
  return result;
}

}  // namespace swerve
}  // namespace ocs2