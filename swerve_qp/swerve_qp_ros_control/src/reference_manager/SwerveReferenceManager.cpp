#include "reference_manager/SwerveReferenceManager.hpp"

#include <bitset>
#include <iostream>

namespace ocs2 {
namespace swerve {

// initialize with brakes engaged ("1111")
SwerveReferenceManager::SwerveReferenceManager(SwerveModelInfo swerveModelInfo, NominalSteeringActive nominalSteeringActive)
    : ReferenceManager(TargetTrajectories(),
                       ModeSchedule(std::vector<scalar_t>(), std::vector<size_t>{std::bitset<4>("1111").to_ullong()})),
      swerveModelInfo_(swerveModelInfo),
      nominalSteeringActive_(nominalSteeringActive),
      activate_arm{false, false, false, false, false, false},
      activateSteer_(swerveModelInfo_.brakeDim, false) {}

Eigen::VectorXd SwerveReferenceManager::getNominalSteerActive() const {
  Eigen::VectorXd activedSteerVec = Eigen::VectorXd::Zero(activateSteer_.size());

  if (nominalSteeringActive_ == NominalSteeringActive::Deactive) {
    activedSteerVec.setZero();
  } else if (nominalSteeringActive_ == NominalSteeringActive::Active) {
    activedSteerVec.setOnes();
  } else if (nominalSteeringActive_ == NominalSteeringActive::AngleDependend) {
    // Activated if initial state bigger than given angle
    for (size_t i = 0; i < swerveModelInfo_.brakeDim; ++i) {
      if (activateSteer_[i]) {
        activedSteerVec[i] = 1.0;
      }
    }
  }
  return activedSteerVec;
}

Eigen::VectorXi SwerveReferenceManager::getBrakesState(scalar_t time) const {
  // auto modeSchedule = getModeSchedule().modeAtTime(time);
  // std::bitset<sizeof(size_t)> brakeStates(getModeSchedule().modeAtTime(time));
  Eigen::VectorXi brakeStateVec(swerveModelInfo_.brakeDim);
  // for (size_t i = 0; i < swerveModelInfo_.brakeDim; ++i) {
  //   brakeStateVec[i] = brakeStates[i];
  // }
  return brakeStateVec;
}

void SwerveReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                              TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  for (int i = 0; i < swerveModelInfo_.steerDim; i++) {
    if (initState(modelSettings_.steersStateIndex[i]) > 2.4 || initState(modelSettings_.steersStateIndex[i]) < -2.4) {
      activateSteer_[i] = true;
    } else {
      activateSteer_[i] = false;
    }
  }

  if (swerveModelInfo_.armPresent) {
    for (int i = 0; i < swerveModelInfo_.armDim; i++) {
      if (initState(modelSettings_.armJointsStateIndex[i]) > modelSettings_.armLimits.armUpperLimits[i] ||
          initState(modelSettings_.armJointsStateIndex[i]) < modelSettings_.armLimits.armLowerLimits[i]) {
        activate_arm[i] = true;
      } else {
        activate_arm[i] = false;
      }
    }
  }
}
}  // namespace swerve
}  // namespace ocs2
