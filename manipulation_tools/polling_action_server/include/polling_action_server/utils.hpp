/*
 * utils.hpp
 *
 *  Created on: 27 Mar 2020
 *      Author: Perry Franklin
 */

#pragma once

#include <string>

namespace polling_action_server {

enum class ServerState : char { SHUTDOWN = 0, PAUSED = 1, ABORTED = 2, SUCCEEDED = 3, PREEMPTED = 4, ACTIVE = 5, IDLE = 6 };

}  // namespace polling_action_server

namespace std {
std::ostream& operator<<(std::ostream& os, polling_action_server::ServerState serverState);
}  // namespace std
