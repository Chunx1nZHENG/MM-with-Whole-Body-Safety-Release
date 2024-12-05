/*
 * TrapezoidalVelocityProfile.hpp
 *
 *  Created on: 31 Mar 2020
 *      Author: Perry Franklin
 */

#pragma once

namespace trapezoidal_tracker {

/* Represents a trapezoidal velocity profile (linear velocity ramp up to maxVelocity, plateau, linear ramp to 0)
 * This always ramps from s = 0 to s = 1, ramping up velocity at the specified acceleration
 * The start time is always 0, and the end time is the minimum possible.
 * Max velocity should be given in ds
 */
class TrapezoidalVelocityProfile {
public:
  /**\brief Constructor
   * \param[in] maxVelocity           : Maximum velocity in s space
   * \param[in] acceleration          : acceleration
   */
  TrapezoidalVelocityProfile(double maxVelocity, double acceleration);
  virtual ~TrapezoidalVelocityProfile() = default;

  /**\brief Gets the position, velocity, and acceleration of s at a given timePoint
   * \param[in] timePoint           : Timepoint to get the state at (ramp begins at timePoint=0)
   * \param[out] sOut               : value of s (position)
   * \param[out] dsOut              : value of derivative of s (velocity)
   * \param[out] ddsOut             : value of second derivative of s (acceleration)
   */
  void getState(const double timePoint, double& sOut, double& dsOut, double& ddsOut) const;


  /**\brief Gets the final time of the profile (ie when s=1, ds=0, dds=0)
   * \return ending timepoint (seconds)
   */
  double getEndTimePoint() const;

private:

  const double maxVelocity_;
  const double acceleration_;

  const double startTimePoint_ = 0.0;
  double plateauTimePoint_;
  double rampDownTimePoint_;
  double endTimePoint_;

  double sAtPlateauTimePoint_;
};

} // namespace trapezoidal_tracker
