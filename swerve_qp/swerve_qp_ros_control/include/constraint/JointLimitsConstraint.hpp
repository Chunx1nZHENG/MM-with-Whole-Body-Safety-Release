#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include "SwerveModelInfo.hpp"

namespace ocs2{
namespace swerve{

class JointLimitsConstraint final : public StateConstraint{
 public:
  JointLimitsConstraint(SwerveModelInfo swerveModelInfo) : StateConstraint(ConstraintOrder::Linear), swerveModelInfo_(swerveModelInfo){}
  ~JointLimitsConstraint() override = default;
  JointLimitsConstraint* clone() const override {return new JointLimitsConstraint(*this);}

  size_t getNumConstraints(scalar_t time) const override {return swerveModelInfo_.jointLimitsDim;}
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;


 private:
  JointLimitsConstraint(const JointLimitsConstraint& rhs) : StateConstraint(rhs), swerveModelInfo_(rhs.swerveModelInfo_){};
  SwerveModelInfo swerveModelInfo_;
};
  
} // namespace swerve
} // namespace ocs2
