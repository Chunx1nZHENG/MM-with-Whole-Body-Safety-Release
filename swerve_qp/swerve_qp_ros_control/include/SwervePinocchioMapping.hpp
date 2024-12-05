#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "SwerveModelInfo.hpp"

namespace ocs2 {
namespace swerve {

template <typename SCALAR>
class SwervePinocchioMappingTpl;

using SwervePinocchioMapping = SwervePinocchioMappingTpl<scalar_t>;
using SwervePinocchioMappingCppAd = SwervePinocchioMappingTpl<ad_scalar_t>;

template <typename SCALAR>
class SwervePinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit SwervePinocchioMappingTpl(SwerveModelInfo swerveModelInfo);

  ~SwervePinocchioMappingTpl() override = default;
  SwervePinocchioMappingTpl<SCALAR>* clone() const override;

  /**
   * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state.
   *
   * @param [in] state: system state vector
   * @return pinocchio joint positions, which are also the robot's generalized positions with a ZYX-Euler angle
   * parameterization of the base orientation
   */
  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  /**
   * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input.
   *
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return pinocchio joint velocities, which are also the time derivatives of the pinocchio joint positions
   */
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   *
   * @param [in] state: system state vector
   * @param [in] Jq: jacobian with respect to pinocchio joint positions
   * @param [in] Jv: jacobian with respect to pinocchio joint velocities
   * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
   */
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /**
   * Returns the mobile manipulator model info.
   */
  const SwerveModelInfo& getSwerveModelInfo() const { return swerveModelInfo_; }

 private:
  SwervePinocchioMappingTpl(const SwervePinocchioMappingTpl& rhs) = default;

  const SwerveModelInfo swerveModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2