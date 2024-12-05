/*
 * TrapezoidalVelocityProfile.cpp
 *
 *  Created on: 31 Mar 2020
 *      Author: Perry Franklin
 */

#include <trapezoidal_tracker/TrapezoidalVelocityProfile.hpp>

#include <stdexcept>
#include <cmath>
#include <iostream>

namespace trapezoidal_tracker {

TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(double maxVelocity, double acceleration):
  maxVelocity_(maxVelocity), acceleration_(acceleration){

  if (maxVelocity <= 0){
    throw std::runtime_error("max velocity must be greater than 0!");
  }
  if (acceleration <= 0){
    throw std::runtime_error("acceleration must be greater than 0!");
  }


  const double rampTime = maxVelocity/acceleration;

  // Assuming no ramp up
  const double timeAtMaxVelocity = 1.0/maxVelocity;

  // Two cases: we don't need to reach the max velocity/ we reach max velocity and plateau for a bit
  if (timeAtMaxVelocity < rampTime){
    // no plateau

    // Geometrically, the trapezoid becomes a triangle
    // The height of the triangle is time/2 * acceleration
    // The base is equal to time, so the area is acceleration*time/2*time/2, which must be 1
    // 1 = acceleration*time*time/4
    // 4 /acceleration = time*time
    // sqrt(4/acceleration) = time
    endTimePoint_ = sqrt(4.0/acceleration);

    plateauTimePoint_ = endTimePoint_/2.0;
    rampDownTimePoint_ = plateauTimePoint_;

  } else {
    // plateau

    // Geometrically, the position is the area of the trapezoid.
    // The first triangle has a base of length rampTime
    // The vertex

    plateauTimePoint_ = rampTime;
    rampDownTimePoint_ = timeAtMaxVelocity;
    endTimePoint_ = timeAtMaxVelocity + rampTime;
  }

  sAtPlateauTimePoint_ = acceleration_*pow(plateauTimePoint_ - startTimePoint_, 2)/2.0;
}


void TrapezoidalVelocityProfile::getState(const double timePoint, double& sOut, double& dsOut, double& ddsOut) const{
  if (timePoint > endTimePoint_){
    // After finish, constant at s=1
    sOut = 1.0;
    dsOut = 0.0;
    ddsOut = 0.0;
  } else if (timePoint > rampDownTimePoint_){
    // During ramp down, end at s=1 ds=0, linear ramp down in velocity
    sOut = 1 - acceleration_*pow(timePoint - endTimePoint_, 2)/2.0;
    dsOut = -acceleration_*(timePoint - endTimePoint_);
    ddsOut = -acceleration_;
  } else if (timePoint > plateauTimePoint_){
    // During plateau, s starts at sAtPlateauStart_, velocity constant ds = maxVelocity
    sOut = sAtPlateauTimePoint_ + maxVelocity_*(timePoint - plateauTimePoint_);
    dsOut = maxVelocity_;
    ddsOut = 0.0;
  } else if (timePoint > startTimePoint_){
    // During first ramp, start at s=0 ds=0, linear ramp up in velocity
    sOut = acceleration_*pow(timePoint - startTimePoint_, 2)/2.0;
    dsOut = acceleration_*(timePoint - startTimePoint_);
    ddsOut = acceleration_;
  } else {
    // Before start, constant at s=0
    sOut = 0.0;
    dsOut = 0.0;
    ddsOut = 0.0;
  }
}



double TrapezoidalVelocityProfile::getEndTimePoint() const{
  return endTimePoint_;
}

} // namespace trapezoidal_tracker