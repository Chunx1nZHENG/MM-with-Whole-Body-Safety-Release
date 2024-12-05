#include <string>

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "SwerveInterface.hpp"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/penalties/DoubleSidedPenalty.h>
#include <ocs2_core/penalties/penalties/QuadraticPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include "SwerveDynamics.hpp"
#include "SwervePinocchioMapping.hpp"
#include "constraint/EndEffectorConstraint.hpp"
#include "constraint/JointLimitsConstraint.hpp"
#include "constraint/JointVelocityLimits.hpp"
#include "constraint/LegsConstraintCppAd.hpp"
#include "constraint/LegsInputConstraints.hpp"
#include "constraint/WheelRollingConstraintCppAd.hpp"
#include "cost/NominalArmStateCppAd.hpp"
#include "cost/NominalSteerStateCppAd.hpp"
#include "cost/QuadraticInputCost.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace swerve {

SwerveInterface::SwerveInterface(const std::string& taskFile, const std::string& libraryFolder, const std::string& urdfString,
                                 bool loadSettingsOnly) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[SwerveInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[SwerveInterface] Task file not found: " + taskFilePath.string());
  }

  // load swerve model info
  swerveModelInfo_ = loadSwerveModelInfo(taskFile);

  // Default initial state
  initialState_.resize(swerveModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

  // DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

  if (!loadSettingsOnly) {
    // create library folder if it does not exist
    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    std::cerr << "[SwerveInterface] Generated library path: " << libraryFolderPath << std::endl;

    // create pinocchio interface
    pinocchioInterfacePtr_.reset(new PinocchioInterface(buildPinocchioInterface(urdfString)));
    std::cout << *pinocchioInterfacePtr_;

    int nominalSteerActive = 1;
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::cerr << "\n #### Nominal Steering Active: ";
      std::cerr << "\n #### =============================================================================\n";
      loadData::loadPtreeValue(pt, nominalSteerActive, "quadraticPenalty.nominalSteerActive", true);
      std::cerr << " #### =============================================================================\n";
    }

    // Reference Manager
    referenceManagerPtr_.reset(
        new SwerveReferenceManager(swerveModelInfo_, SwerveReferenceManager::NominalSteeringActive(nominalSteerActive)));
    
    if (referenceManagerPtr_ == nullptr) {
      throw std::runtime_error("swerveInterface ReferenceManager pointer is nullptr!");
    }
    else{
      std::cerr<< "referenceManagerPtr_ is not nullptr"<<std::endl;

    }
    /*
    
     * Optimal control problem
     */
    // Cost
    problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));
    problem_.softConstraintPtr->add(
        "wheelRollingCost", getQuadraticWheelRollingCost(taskFile, *pinocchioInterfacePtr_, "wheelRollingCost", libraryFolder, true));
    problem_.softConstraintPtr->add("jointVelocityLimit", getJointVelocityLimitConstraint(taskFile));
    problem_.stateSoftConstraintPtr->add("jointLimits", getJointLimitsConstraint(taskFile));
    // problem_.stateSoftConstraintPtr->add("legsCost", getQuadraticLegsStateCost(*pinocchioInterfacePtr_, "legsCost", libraryFolder, true));
    // problem_.equalityConstraintPtr->add(
    //     "legs_input_constraints", std::unique_ptr<StateInputConstraint>(new LegsInputConstraints(swerveModelInfo_, *referenceManagerPtr_)));

    problem_.stateSoftConstraintPtr->add("nominal_steer_cost", getNominalSteerCost(taskFile, "nominal_steer_cost", libraryFolder, true));

    if (swerveModelInfo_.armPresent) {
      for (int i = 0; i < swerveModelInfo_.armDim; ++i) {
        problem_.stateSoftConstraintPtr->add("nominal_arm_cost" + std::to_string(i),
                                             getNominalArmCost(taskFile, "nominal_arm_cost_" + std::to_string(i), libraryFolder, true, i));
      }
    }

    problem_.stateSoftConstraintPtr->add(
        "endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector", false, libraryFolder, true));
  }
  // Dynamics
  problem_.dynamicsPtr.reset(new SwerveDynamics(swerveModelInfo_, "swerve_dynamics", libraryFolder, true, true));

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(swerveModelInfo_.inputDim));
}

SwerveModelInfo SwerveInterface::loadSwerveModelInfo(const std::string& taskFile) {
  SwerveModelInfo swerveModelInfo;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  // load variables
  loadData::loadPtreeValue(pt, swerveModelInfo.armPresent, "model_information.armPresent", true);
  loadData::loadPtreeValue(pt, swerveModelInfo.eeFrame, "model_information.eeFrame", true);

  if (swerveModelInfo.armPresent) {
    swerveModelInfo.stateDim = swerveModelInfo.steerDim + swerveModelInfo.wheelDim  + swerveModelInfo.armDim + 7;
    swerveModelInfo.inputDim = swerveModelInfo.steerDim + swerveModelInfo.wheelDim  + swerveModelInfo.armDim + 3;
    swerveModelInfo.pinnochioInputDim =
        swerveModelInfo.steerDim + swerveModelInfo.wheelDim  + swerveModelInfo.armDim + 6;
    swerveModelInfo.jointLimitsDim = swerveModelInfo.steerDim + swerveModelInfo.armDim;

  } else {
    swerveModelInfo.stateDim = swerveModelInfo.steerDim + swerveModelInfo.wheelDim  + 7;
    swerveModelInfo.inputDim = swerveModelInfo.steerDim + swerveModelInfo.wheelDim  + 3;
    swerveModelInfo.pinnochioInputDim = swerveModelInfo.steerDim + swerveModelInfo.wheelDim + 6;
    swerveModelInfo.jointLimitsDim = swerveModelInfo.steerDim;
  }

  return swerveModelInfo;
}

PinocchioInterface SwerveInterface::buildPinocchioInterface(const std::string& urdfString) {
  pinocchio::JointModelFreeFlyer rootJoint;
  return getPinocchioInterfaceFromUrdfString(urdfString, rootJoint);
}

std::unique_ptr<StateCost> SwerveInterface::getQuadraticLegsStateCost(const PinocchioInterface& pinocchioInterface,
                                                                      const std::string& prefix, const std::string& libraryFolder,
                                                                      bool recompileLibraries) {
  std::unique_ptr<StateConstraint> constraint;
  SwervePinocchioMappingCppAd pinocchioMapping(swerveModelInfo_);
  constraint.reset(new LegsConstraintCppAd(prefix, swerveModelInfo_, libraryFolder, recompileLibraries, *referenceManagerPtr_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(4);
  std::generate_n(penaltyArray.begin(), 4, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(100)); });
  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateInputCost> SwerveInterface::getQuadraticInputCost(const std::string& taskFile) {
  matrix_t R(swerveModelInfo_.inputDim, swerveModelInfo_.inputDim);

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadEigenMatrix(taskFile, "inputCost.R", R);
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::unique_ptr<StateInputCost>(new QuadraticInputCost(std::move(R), swerveModelInfo_.stateDim));
}

std::unique_ptr<StateInputCost> SwerveInterface::getQuadraticWheelRollingCost(const std::string& taskFile,
                                                                              const PinocchioInterface& pinocchioInterface,
                                                                              const std::string& prefix, const std::string& libraryFolder,
                                                                              bool recompileLibraries) {
  scalar_t muWheel = 1.0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### Wheel Rolling: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muWheel, "quadraticPenalty.wheelRolling", true);
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateInputConstraint> constraint;
  SwervePinocchioMappingCppAd pinocchioMapping(swerveModelInfo_);
  constraint.reset(
      new WheelRollingConstraintCppAd(pinocchioInterface, pinocchioMapping, prefix, libraryFolder, recompileLibraries, swerveModelInfo_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(8);
  std::generate_n(penaltyArray.begin(), 8, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muWheel)); });
  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateCost> SwerveInterface::getNominalSteerCost(const std::string& taskFile, const std::string& prefix,
                                                                const std::string& libraryFolder, bool recompileLibraries) {
  scalar_t muSteer = 1.0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### Nominal Steer: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muSteer, "quadraticPenalty.nominalSteerState", true);
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateConstraint> constraint;
  constraint.reset(new NominalSteerStateCppAd(prefix, swerveModelInfo_, *referenceManagerPtr_, libraryFolder, recompileLibraries));
  std::unique_ptr<PenaltyBase> penalty;
  penalty.reset(new QuadraticPenalty(muSteer));
  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

std::unique_ptr<StateCost> SwerveInterface::getNominalArmCost(const std::string& taskFile, const std::string& prefix,
                                                              const std::string& libraryFolder, bool recompileLibraries, int index) {
  scalar_t muSteer = 1.0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### Nominal Arm: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muSteer, "quadraticPenalty.nominalArmState", true);
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateConstraint> constraint;
  constraint.reset(new NominalArmStateCppAd(prefix, swerveModelInfo_, *referenceManagerPtr_, libraryFolder, recompileLibraries, index));
  std::unique_ptr<PenaltyBase> penalty;
  penalty.reset(new QuadraticPenalty(muSteer));
  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

std::unique_ptr<StateCost> SwerveInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                     const std::string& taskFile, const std::string& prefix,
                                                                     bool usePreComputation, const std::string& libraryFolder,
                                                                     bool recompileLibraries) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    SwervePinocchioMapping pinocchioMapping(swerveModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {swerveModelInfo_.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } else {
    SwervePinocchioMappingCppAd pinocchioMappingCppAd(swerveModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {swerveModelInfo_.eeFrame},
                                                     swerveModelInfo_.stateDim, swerveModelInfo_.inputDim, "end_effector_kinematics",
                                                     libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateInputCost> SwerveInterface::getJointVelocityLimitConstraint(const std::string& taskFile) {
  vector_t lowerBound(swerveModelInfo_.inputDim);
  vector_t upperBound(swerveModelInfo_.inputDim);
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "jointVelocityLimits.";
  std::cerr << "\n #### JointVelocityLimits Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound", lowerBound);
  std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound", upperBound);
  std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
  loadData::loadPtreeValue(pt, mu, prefix + "mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + "delta", true);
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateInputConstraint> constraint(new JointVelocityLimits(swerveModelInfo_.inputDim));

  std::unique_ptr<PenaltyBase> barrierFunction;
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(swerveModelInfo_.inputDim);
  for (int i = 0; i < swerveModelInfo_.inputDim; i++) {
    barrierFunction.reset(new RelaxedBarrierPenalty({mu, delta}));
    penaltyArray[i].reset(new DoubleSidedPenalty(lowerBound(i), upperBound(i), std::move(barrierFunction)));
  }

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

std::unique_ptr<StateCost> SwerveInterface::getJointLimitsConstraint(const std::string& taskFile) {
  vector_t lowerLimit(swerveModelInfo_.jointLimitsDim);
  vector_t upperLimit(swerveModelInfo_.jointLimitsDim);
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "jointLimits.";
  std::cerr << "\n #### JointLimits Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadEigenMatrix(taskFile, prefix + "lowerLimit", lowerLimit);
  std::cerr << " #### 'lowerLimit':  " << lowerLimit.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, prefix + "upperLimit", upperLimit);
  std::cerr << " #### 'upperLimit':  " << upperLimit.transpose() << std::endl;
  loadData::loadPtreeValue(pt, mu, prefix + "mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + "delta", true);
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateConstraint> constraint(new JointLimitsConstraint(swerveModelInfo_));

  std::unique_ptr<PenaltyBase> barrierFunction;
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(swerveModelInfo_.jointLimitsDim);
  for (int i = 0; i < swerveModelInfo_.jointLimitsDim; i++) {
    barrierFunction.reset(new RelaxedBarrierPenalty({mu, delta}));
    penaltyArray[i].reset(new DoubleSidedPenalty(lowerLimit(i), upperLimit(i), std::move(barrierFunction)));
  }

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

}  // namespace swerve
}  // namespace ocs2
