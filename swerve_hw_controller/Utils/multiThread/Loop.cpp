#include "Loop.h"
#include <sys/timerfd.h>
#include <unistd.h>
#include <cmath>
#include <iostream>

Loop::Loop(std::string name, float period, int bindCPU)
  :_name(name), _period(period), _bindCPU(bindCPU){
  _timer = new AbsoluteTimer(_period);
}

Loop::~Loop(){
  shutdown(); 
  delete _timer;
}

void Loop::start() {
  if (_isrunning) {
    printf("[Error] Loop %s is already running.\n", _name.c_str());
    return;
  }

  printf("[Loop Start] named: %s, period: %d(ms), ", _name.c_str(), (int)(_period*1000.0));
  if(_bindCPU > 0){
    _bind_cpu_flag = true;
    printf("run at cpu: %d \n", _bindCPU);
  } else {
    _bind_cpu_flag = false;
    printf("cpu unspecified\n");
  }

  _isrunning = true;
  _thread = std::thread(&Loop::entryFunc, this);
  if(_bind_cpu_flag == true) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(_bindCPU, &cpuset);
      if(0 != pthread_setaffinity_np(_thread.native_handle(), sizeof(cpu_set_t), &cpuset)){
        printf("Error: Set affinity failed.\n");
        // exit(-2);
    }
  }
}

void Loop::shutdown() {
  if (!_isrunning) {
    printf("[Warning] Loop %s shutdown nothing.\n", _name.c_str());
    return;
  }
  _isrunning = false;
  // printf("[Loop] Waiting for %s to shutdown...\n", _name.c_str());
  _thread.join();
  // std::cout << "[REPORT] The time out rate of thread " << _name << " is " << 100.0*(double)_timeOutTimes/(double)_runTimes << "%" << std::endl;
  return;
  // printf("[Loop] Done!\n");
}

void Loop::entryFunc() {
  while (_isrunning) {
    _timer->start();
    ++_runTimes;
    functionCB();
    if(!_timer->wait()){
      ++_timeOutTimes;
      // std::cout << "[WARNING] Loop " << _name << " time out." << std::endl;
    }
  }

  printf("[Loop End] named: %s\n", _name.c_str());
}
