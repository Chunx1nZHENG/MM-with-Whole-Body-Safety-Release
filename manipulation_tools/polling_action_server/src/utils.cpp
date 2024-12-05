/*
 * utils.cpp
 *
 *  Created on: 27 Mar 2020
 *      Author: Perry Franklin
 */

#include <iostream>

#include <polling_action_server/utils.hpp>

namespace std {

std::ostream& operator<<(std::ostream& os, polling_action_server::ServerState serverState) {
  switch (serverState) {
    case polling_action_server::ServerState::SHUTDOWN:
      os << "[SHUTDOWN]";
      break;
    case polling_action_server::ServerState::PAUSED:
      os << "[PAUSED]";
      break;
    case polling_action_server::ServerState::ABORTED:
      os << "[ABORTED]";
      break;
    case polling_action_server::ServerState::SUCCEEDED:
      os << "[SUCCEEDED]";
      break;
    case polling_action_server::ServerState::PREEMPTED:
      os << "[PREEMPTED]";
      break;
    case polling_action_server::ServerState::ACTIVE:
      os << "[ACTIVE]";
      break;
    case polling_action_server::ServerState::IDLE:
      os << "[IDLE]";
      break;
    default:
      os << "[Unknown State]";
      break;
  }
  return os;
}

}  // namespace std
