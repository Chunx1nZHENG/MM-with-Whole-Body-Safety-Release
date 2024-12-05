#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include "SwerveModelInfo.hpp"
#include "reference_manager/SwerveReferenceManager.hpp"

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {

namespace swerve {

class SwerveInterface final : public RobotInterface {
 public:
  explicit SwerveInterface(const std::string& taskFile, const std::string& libraryFolder, const std::string& urdfString,
                           bool loadSettingsOnly = false);

  const vector_t& getInitialState() { return initialState_; }

  ddp::Settings& ddpSettings() { return ddpSettings_; }

  mpc::Settings& mpcSettings() { return mpcSettings_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  std::shared_ptr<SwerveReferenceManager> getSwerveReferenceManagerPtr() const { return referenceManagerPtr_; }

  const Initializer& getInitializer() const override { return *initializerPtr_; }

  const RolloutBase& getRollout() const { return *rolloutPtr_; }

  const PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_; }

  static PinocchioInterface buildPinocchioInterface(const std::string& urdfString);

  const SwerveModelInfo getSwerveModelInfo() { return swerveModelInfo_; }

 private:
  SwerveModelInfo loadSwerveModelInfo(const std::string& taskFile);
  std::unique_ptr<StateInputCost> getQuadraticInputCost(const std::string& taskFile);
  std::unique_ptr<StateInputCost> getQuadraticWheelRollingCost(const PinocchioInterface& pinocchioInterface, const std::string& prefix,
                                                               const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateInputCost> getQuadraticWheelRollingCost(const std::string& taskFile, const PinocchioInterface& pinocchioInterface,
                                                               const std::string& prefix, const std::string& libraryFolder,
                                                               bool recompileLibraries);
  std::unique_ptr<StateCost> getNominalSteerCost(const std::string& taskFile, const std::string& prefix, const std::string& libraryFolder,
                                                 bool recompileLibraries);
  std::unique_ptr<StateCost> getNominalArmCost(const std::string& taskFile, const std::string& prefix, const std::string& libraryFolder,
                                               bool recompileLibraries, int index);
  std::unique_ptr<StateCost> getQuadraticLegsStateCost(const PinocchioInterface& pinocchioInterface, const std::string& prefix,
                                                       const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                      const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                      bool recompileLibraries);
  std::unique_ptr<StateInputCost> getJointVelocityLimitConstraint(const std::string& taskFile);
  std::unique_ptr<StateCost> getJointLimitsConstraint(const std::string& taskFile);
  std::unique_ptr<StateInputConstraint> getWheelRollingConstraint(const PinocchioInterface& pinocchioInterface, const std::string& prefix,
                                                                  const std::string& libraryFolder, bool recompileLibraries);

  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;

  OptimalControlProblem problem_;
  std::shared_ptr<SwerveReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;

  vector_t initialState_;

  SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2