#include "constraint/LegsInputConstraints.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>

namespace ocs2 {
namespace swerve {

LegsInputConstraints::LegsInputConstraints(SwerveModelInfo swerveModelInfo, const SwerveReferenceManager& referenceManager)
    : StateInputConstraint(ConstraintOrder::Linear), referenceManagerPtr_(&referenceManager), swerveModelInfo_(swerveModelInfo) {}

LegsInputConstraints::LegsInputConstraints(const LegsInputConstraints& rhs)
    : StateInputConstraint(rhs), referenceManagerPtr_(rhs.referenceManagerPtr_), swerveModelInfo_(rhs.swerveModelInfo_) {}

LegsInputConstraints* LegsInputConstraints::clone() const {
  return new LegsInputConstraints(*this);
}

vector_t LegsInputConstraints::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                        const PreComputation& /* preComputation */) const {
  // Eigen::VectorXi brakesStates = referenceManagerPtr_->getBrakesState(time);
  vector_t value(4);
  // size_t j = 0;
  // for (size_t i = 0; i < swerveModelInfo_.brakeDim; ++i) {
  //   if (brakesStates(i) > 0) {
  //     value(j) = input(modelSettings_.brakesInputIndex[i]);
  //     j++;
  //   }
  // }
  return value;
}

VectorFunctionLinearApproximation LegsInputConstraints::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                               const PreComputation& /* preComputation */) const {
  // Eigen::VectorXi brakesStates = referenceManagerPtr_->getBrakesState(time);
  VectorFunctionLinearApproximation jac;
  // jac.f = vector_t(brakesStates.sum());
  // jac.dfdx = matrix_t::Zero(brakesStates.sum(), state.size());
  // jac.dfdu = matrix_t::Zero(brakesStates.sum(), input.size());
  // size_t j = 0;
  // for (size_t i = 0; i < swerveModelInfo_.brakeDim; ++i) {
  //   if (brakesStates(i) > 0) {
  //     jac.f(j) = input(modelSettings_.brakesInputIndex[i]);
  //     jac.dfdu(j, modelSettings_.brakesInputIndex[i]) = 1;
  //     j++;
  //   }
  // }
  return jac;
}

size_t LegsInputConstraints::getNumConstraints(scalar_t time) const {
  // return referenceManagerPtr_->getBrakesState(time).sum();
  return 1;
}

}  // namespace swerve
}  // namespace ocs2