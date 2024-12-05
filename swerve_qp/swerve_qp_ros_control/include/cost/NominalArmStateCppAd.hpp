#pragma once

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraintCppAd.h>
#include "SwerveModelInfo.hpp"
#include "reference_manager/SwerveReferenceManager.hpp"

namespace ocs2 {
namespace swerve {

class NominalArmStateCppAd final : public StateConstraintCppAd {
 public:
  NominalArmStateCppAd(const std::string& prefix, SwerveModelInfo swerveModelInfo, const SwerveReferenceManager& referenceManager,
                       const std::string& libraryFolder, bool recompileLibraries, int index);
  ~NominalArmStateCppAd() override = default;

  NominalArmStateCppAd* clone() const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getParameters(scalar_t time) const override { return vector_t(0); }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;
  bool isActive(scalar_t time) const override;

 private:
  NominalArmStateCppAd(const NominalArmStateCppAd& rhs);
  int index_;
  ModelSettings modelSettings_;
  const SwerveReferenceManager* referenceManagerPtr_;
  SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2