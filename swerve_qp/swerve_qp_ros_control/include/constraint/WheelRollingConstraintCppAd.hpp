#pragma once

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include "SwerveModelInfo.hpp"
#include "SwervePinocchioMapping.hpp"

#include <ocs2_pinocchio_interface/pinocchio_forward_declaration.h>

namespace ocs2 {
namespace swerve {

class WheelRollingConstraintCppAd final: public StateInputConstraintCppAd {
  using ad_vector3 = Eigen::Matrix<ad_scalar_t, 3, 1>;
 public:
  WheelRollingConstraintCppAd(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                              const std::string& prefix, const std::string& libraryFolder, bool recompileLibraries, SwerveModelInfo swerveModelInfo);
  ~WheelRollingConstraintCppAd() override = default;

  WheelRollingConstraintCppAd* clone() const override;
  size_t getNumConstraints(scalar_t time) const override { return 8; }
  vector_t getParameters(scalar_t time) const { return vector_t(0); }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;
 protected:
  WheelRollingConstraintCppAd(const WheelRollingConstraintCppAd& rhs);

 private:
  const PinocchioInterface* pinocchioInterfacePtr_;
  std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr_;
  SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2
