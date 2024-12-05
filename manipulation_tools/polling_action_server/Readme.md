# polling_action_server

## Overview

Provides a simplified polling variety of SimpleActionServer.

SimpleActionServers require users to created their own executeFunction, which is run in a separate thread. This means servers that use SimpleActionServers will need to be multithreaded, which can add a large amount of complexity to the system.

PollingActionServer instead exposes only functions that can be run in our standard ```advance()``` functions, and abstract away the multithreading safety. The goal is to reduce complexity in classes that already run some form of execution loop. This PollingActionServer can be used by a SimpleActionClient, as well as the axclient from action_lib.

## Usage

You must instantiate a ```PollingActionServer<YourActionType>```, the same way that you would instantiate a SimpleActionServer. You must specify an ```actionNamespace```, which will correspond to rostopics called ```actionNamespace/goal, actionNamespace/cancel/...```, ie the normal action server topics. You will also need to specify a feedback update period, which determines the rate at which feedback is published as well as the resolution for termination (meaning that the action can only be terminated on each feedback tick, not as soon as the ```setSucceeded(...)``` or equivalent is called).

During each loop, you should check for whether a new goal has been received. Calling ```getGoal(goal)``` will return whether the goal has already been gotten (calling this function means future calls will return false until a new goal is received by the action server). To just check if a new goal is available, you can call ```haveNewGoal()```. If you want to check the goal but still leave it marked as a new goal, you can call ```getGoalSilently(goal)```.

During execution, you can use ```setFeedback(feedback)``` to provide feedback to the action server. This feedback is only published at the rate you specified, so not all feedback may be published.

When the goal is reached, you must call ```setSucceeded(result)``` with a final result to return. This will cause the ```PollingActionServer``` to terminate and return the result to the calling action client, and make the action server available for other requests.

If the goal fails, or needs to be preempted, call ```setAborted(reason, result)``` or ```setPreempted(reason, result)```. You are required to provide a ```reason```, so make it a good message for debugging!

If you want to allow preempting, you can call ```getPreemptRequested()``` to check if a preempt has been received by the action server. You may then handle the preempt as you see fit, and when you are ready to receive the next goal you can call ```setPreempted(reason, result)```.

### Notes

When the ```PollingActionServer``` has received a new goal, it is considered ```ACTIVE``` regardless of whether you are actually using it or not, and will begin publishing feedback.

There is a separate polling_action_server::ServerState, which does not directly correspond to the actionlib::ServerState. In particular, actionlib can define multiple more states, such as Pending, Rejected, and some others. The polling_action_server::ServerState refers only to the state of the server exposed via public functions. For instance, even if you have not acquired the most recent goal the state of the action server will be ACTIVE.

The action server can receive an external preempt in two ways: the original client has cancelled the goal, or a different client has sent a more recent goal to the server.