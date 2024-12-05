#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "SwerveModelInfo.hpp"
#include "BrakesMode.h"

namespace ocs2 {
namespace swerve {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwerveReferenceManager : public ReferenceManager {
 public:
  enum class NominalSteeringActive : int { Deactive = 0, Active = 1, AngleDependend = 2 };

  SwerveReferenceManager(SwerveModelInfo swerveModelInfo, NominalSteeringActive nominalSteeringActive);

  ~SwerveReferenceManager() override = default;

  Eigen::VectorXi getBrakesState(scalar_t time) const;
  Eigen::VectorXd getNominalSteerActive() const;
  bool getNominalArmActive(int index) const;

  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

 private:
  ModelSettings modelSettings_;
  SwerveModelInfo swerveModelInfo_;
  NominalSteeringActive nominalSteeringActive_;
  std::vector<bool> activateSteer_;
  bool activate_arm[6];
};

}  // namespace swerve
}  // namespace ocs2
