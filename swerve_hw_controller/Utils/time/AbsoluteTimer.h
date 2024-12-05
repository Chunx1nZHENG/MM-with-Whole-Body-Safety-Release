/**
 * @file AbsoluteTimer.h
 * @author Zekun Bian
 * @brief 
 * @version 0.1
 * @date 2022-10-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ABSOLUTETIMER_H
#define ABSOLUTETIMER_H

#include <sys/timerfd.h>
#include <stdint.h>

/*
waitTimeS = 0 means do not care time out
*/
class AbsoluteTimer{
public:
    AbsoluteTimer(double waitTimeS);
    ~AbsoluteTimer();
    void start();
    bool wait();
private:
    void _updateWaitTime(double waitTimeS);
    int _timerFd;
    uint64_t _missed;
    double _waitTime;
    double _startTime;
    double _leftTime;
    double _nextWaitTime;
    itimerspec _timerSpec;
};

#endif  // ABSOLUTETIMER_H