#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "SwerveModelInfo.hpp"

namespace ocs2 {
namespace swerve {

class SwerveDynamics final : public SystemDynamicsBaseAD {
 public:
  using Base = SystemDynamicsBaseAD;

  explicit SwerveDynamics(SwerveModelInfo swerveModelInfo, const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                                     bool recompileLibraries = true, bool verbose = true);
  ~SwerveDynamics() override = default;
  SwerveDynamics* clone() const override { return new SwerveDynamics(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override;

 private:
  SwerveDynamics(const SwerveDynamics& rhs) = default;
  const SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2