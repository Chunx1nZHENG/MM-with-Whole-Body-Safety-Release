#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "SwerveModelInfo.hpp"
#include "reference_manager/SwerveReferenceManager.hpp"

namespace ocs2 {
namespace swerve {

class LegsInputConstraints final : public StateInputConstraint {
 public:
  LegsInputConstraints(SwerveModelInfo swerveModelInfo, const SwerveReferenceManager& referenceManager);
  ~LegsInputConstraints() override = default;

  LegsInputConstraints* clone() const override;
  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& /* preComputation */) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& /* preComputation */) const override;

 private:
  LegsInputConstraints(const LegsInputConstraints& rhs);
  const SwerveReferenceManager* referenceManagerPtr_;
  ModelSettings modelSettings_;
  SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2