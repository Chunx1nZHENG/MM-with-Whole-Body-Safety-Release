#ifndef _UART_HPP_
#define _UART_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <thread>
#include <iostream>
#include <mutex>

#include "Timer.h"
#include "RobotMessage.h"

using namespace std;

static std::mutex uart_mut;
static Timer recordTime;
#define BAUD 115200

void initMsg(ModeMsg *msg);
void enterMotorModeCmdTmotor(JointMsg *msg);
void exitMotorModeCmdTmotor(JointMsg *msg);

void prepareWheelHubDrive1(JointMsg *msg);
void prepareWheelHubDrive2(JointMsg *msg);
void prepareWheelHubDrive3(JointMsg *msg);
void prepareWheelHubDrive4(JointMsg *msg);
void setSynchriniusControl4Wheelhub(JointMsg *msg);
void setVelocityMode4Wheelhub(JointMsg *msg);
void setLeftWheelhubAccTime(JointMsg *msg);
void setLeftWheelhubDecTime(JointMsg *msg);
void setRightWheelhubAccTime(JointMsg *msg);
void setRightWheelhubDecTime(JointMsg *msg);
void readWheelhubVelocity(JointMsg *msg);
void readLeftWheelhubPosition(JointMsg *msg);
void readRightWheelhubPosition(JointMsg *msg);
void exitMotorModeCmdZLWheelhub(JointMsg *msg);

void uartSendTmotor(int uartHandle, ModeMsg *msg);
void uartRecvTmotor(int uartHandle, ModeMsg *state);
void unPackReplyTmotor(uint8_t *msg, ModeState *state);
void uartSendRecvTmotor(int uartHandle, ModeMsg *msg, ModeState *state);
void packCmdTmotor(JointMsg *msg, JointControl ctrl);
void packAllCmdTmotor(ModeMsg *msg, ModeControl ctrl);

void uartSendZLWheelhub(int uartHandle, ModeMsg *msg);
void uartRecvZLWheelhub(int uartHandle, ModeMsg *state);
void unPackReplyZLWheelhubVelocity(uint8_t *msg, ModeState *state);
void unPackReplyZLWheelhubPosition(uint8_t *msg, ModeState *state);
void uartSendRecvZLWheelhub(int uartHandle, ModeMsg *msg, ModeState *state);
void packAllCmdZLWheelhub(ModeMsg *msg, ModeControl ctrl);

float uintToFloat(int xInt, float xMin, float xMax, int bits);
float floatTouint(float x, float xMin, float xMax, int bits);
void showState(ModeState state);

int uart_init(int baud, char *uart_path);
int uart_open(int fd, const char *pathname);
int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop);
int uart_close(int fd);


#endif