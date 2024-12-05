#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {
namespace swerve {

class JointVelocityLimits final : public StateInputConstraint {
 public:
  JointVelocityLimits(size_t stateDim) : StateInputConstraint(ConstraintOrder::Linear), stateDim_(stateDim) {}
  ~JointVelocityLimits() override = default;
  JointVelocityLimits* clone() const override { return new JointVelocityLimits(*this); }

  size_t getNumConstraints(scalar_t time) const override { return stateDim_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation&) const override;

 private:
  JointVelocityLimits(const JointVelocityLimits& rhs) : StateInputConstraint(rhs), stateDim_(rhs.stateDim_){};
  const size_t stateDim_;
};

}  // namespace swerve
}  // namespace ocs2