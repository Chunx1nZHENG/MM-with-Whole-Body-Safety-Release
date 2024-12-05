#include <constraint/JointVelocityLimits.hpp>

namespace ocs2
{
namespace swerve
{

vector_t JointVelocityLimits::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const {
  return input;
}

VectorFunctionLinearApproximation JointVelocityLimits::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation&) const {
  VectorFunctionLinearApproximation limits(input.rows(), state.rows(), input.rows());
  limits.f = input;
  limits.dfdx.setZero();
  limits.dfdu.setIdentity();
  return limits;
}

} // namespace swerve
} // namespace ocs2
