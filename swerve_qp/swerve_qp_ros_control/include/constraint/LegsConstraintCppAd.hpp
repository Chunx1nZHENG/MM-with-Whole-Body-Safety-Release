#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraintCppAd.h>

#include "reference_manager/SwerveReferenceManager.hpp"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {
namespace swerve {

class LegsConstraintCppAd final : public StateConstraintCppAd {
 public:
  LegsConstraintCppAd(const std::string& prefix, SwerveModelInfo swerveModelInfo, const std::string& libraryFolder, bool recompileLibraries,
                      const SwerveReferenceManager& referenceManager);
  ~LegsConstraintCppAd() override = default;

  LegsConstraintCppAd* clone() const override;
  size_t getNumConstraints(scalar_t time) const override { return 4; }
  vector_t getParameters(scalar_t time) const override;
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;

 private:
  LegsConstraintCppAd(const LegsConstraintCppAd& rhs);
  const SwerveReferenceManager* referenceManagerPtr_;
  SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2