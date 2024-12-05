/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace ocs2 {
namespace swerve {

using brakes_mode = std::array<bool, 4>;

enum ModeNumber {  // {LF, RF, LH, RH}
  UNLOCK_ALL = 0,
  RH = 1,
  LH = 2,
  LH_RH = 3,
  RF = 4,
  RF_RH = 5,
  RF_LH = 6,
  RF_LH_RH = 7,
  LF = 8,
  LF_RH = 9,
  LF_LH = 10,
  LF_LH_RH = 11,
  LF_RF = 12,
  LF_RF_RH = 13,
  LF_RF_LH = 14,
  LOCK_ALL = 15,
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline brakes_mode modeNumber2BrakesState(const size_t& modeNumber) {
  brakes_mode brakesState;  // {LB, LF, RB, RF}

  switch (modeNumber) {
    case 0:
      brakesState = brakes_mode{false, false, false, false};
      break;  // 0:  unlocked
    case 1:
      brakesState = brakes_mode{false, false, false, true};
      break;  // 1:  RH
    case 2:
      brakesState = brakes_mode{false, false, true, false};
      break;  // 2:  LH
    case 3:
      brakesState = brakes_mode{false, false, true, true};
      break;  // 3:  RH, LH
    case 4:
      brakesState = brakes_mode{false, true, false, false};
      break;  // 4:  RF
    case 5:
      brakesState = brakes_mode{false, true, false, true};
      break;  // 5:  RF, RH
    case 6:
      brakesState = brakes_mode{false, true, true, false};
      break;  // 6:  RF, LH
    case 7:
      brakesState = brakes_mode{false, true, true, true};
      break;  // 7:  RF, LH, RH
    case 8:
      brakesState = brakes_mode{true, false, false, false};
      break;  // 8:  LF,
    case 9:
      brakesState = brakes_mode{true, false, false, true};
      break;  // 9:  LF, RH
    case 10:
      brakesState = brakes_mode{true, false, true, false};
      break;  // 10: LF, LH
    case 11:
      brakesState = brakes_mode{true, false, true, true};
      break;  // 11: LF, LH, RH
    case 12:
      brakesState = brakes_mode{true, true, false, false};
      break;  // 12: LF, RF
    case 13:
      brakesState = brakes_mode{true, true, false, true};
      break;  // 13: LF, RF, RH
    case 14:
      brakesState = brakes_mode{true, true, true, false};
      break;  // 14: LF, RF, LH
    case 15:
      brakesState = brakes_mode{true, true, true, true};
      break;  // 15: locked
  }

  return brakesState;
}

}  // namespace swerve
}  // end of namespace ocs2
