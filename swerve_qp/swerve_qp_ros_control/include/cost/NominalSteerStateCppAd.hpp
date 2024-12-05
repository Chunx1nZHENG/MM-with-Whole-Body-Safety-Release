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

class NominalSteerStateCppAd final : public StateConstraintCppAd {
 public:
  NominalSteerStateCppAd(const std::string& prefix, SwerveModelInfo swerveModelInfo, const SwerveReferenceManager& referenceManager,
                         const std::string& libraryFolder, bool recompileLibraries);
  ~NominalSteerStateCppAd() override = default;

  NominalSteerStateCppAd* clone() const override;
  size_t getNumConstraints(scalar_t time) const override;
  vector_t getParameters(scalar_t time) const override;
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;

 private:
  NominalSteerStateCppAd(const NominalSteerStateCppAd& rhs);
  ModelSettings modelSettings_;
  SwerveModelInfo swerveModelInfo_;
  const SwerveReferenceManager* referenceManagerPtr_;
};

}  // namespace swerve
}  // namespace ocs2