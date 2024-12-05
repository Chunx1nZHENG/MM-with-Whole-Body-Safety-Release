#ifndef ENUM_CLASS_H
#define ENUM_CLASS_H

enum class RobotFSMState{
    INVALID,
    PASSIVE,
    ACKERMAN,
    LINEARMOVEMENT,
    TURNING
};

enum class RobJointEnum{
    LFET_FRONT,
    LEFT_BACK,
    RIGHT_BACK,
    RIGHT_FRONT,
    BODY
};

#endif  // ENUM_CLASS_H