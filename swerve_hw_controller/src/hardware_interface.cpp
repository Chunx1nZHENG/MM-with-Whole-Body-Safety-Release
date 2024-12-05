#include "swerve_base_controller/hardware_interface.hpp"

#define UART_PATH_FRONT_TMOTOR (char *)"/dev/ttyACM0"
#define UART_PATH_BACK_TMOTOR (char *)"/dev/ttyACM3"
#define UART_PATH_FRONT_ZL (char *)"/dev/ttyACM2"
#define UART_PATH_BACK_ZL (char *)"/dev/ttyACM1"

namespace swerve 
{
HardwareInterface::HardwareInterface()
{
    _zlWheelhubMsgType = 0;

    _uartFrontTmotor = uart_init(BAUD, UART_PATH_FRONT_TMOTOR);
    _uartBackTmotor = uart_init(BAUD, UART_PATH_BACK_TMOTOR);
    _uartFrontZLWheelhub = uart_init(BAUD, UART_PATH_FRONT_ZL);
    _uartBackZLWheelhub = uart_init(BAUD, UART_PATH_BACK_ZL);

    try{
        initMsg(&_frontModeMsg);
        initMsg(&_backModeMsg);
    } catch (const std::exception&) {
        std::cout << "Fail init msg" << std::endl;
        exit(EXIT_FAILURE);
    }

    _initDriveTmotor();
    _initDriveZL();

    _frontTmotorThread = new LoopFunc("front Tmotor sendrecv thread", 0.002, boost::bind(&uartSendRecvTmotor, _uartFrontTmotor, &_frontModeMsg, &_frontModeState));
    _backTmotorThread = new LoopFunc("back Tmotor sendrecv thread", 0.002, boost::bind(&uartSendRecvTmotor, _uartBackTmotor, &_backModeMsg, &_backModeState));
    _frontZLWheelhubThread = new LoopFunc("front zl wheelhub sendrecv thread", 0.002, boost::bind(&uartSendRecvZLWheelhub, _uartFrontZLWheelhub, &_frontModeMsg, &_frontModeState));
    _backZLWheelhubThread = new LoopFunc("back zl wheelhub sendrecv thread", 0.002, boost::bind(&uartSendRecvZLWheelhub, _uartBackZLWheelhub, &_backModeMsg, &_backModeState));

    _frontTmotorThread->start();
    _backTmotorThread->start();
    _frontZLWheelhubThread->start();
    _backZLWheelhubThread->start();

    _kinovaPtr = new KortexRobot("192.168.1.10");
}

HardwareInterface::~HardwareInterface()
{
    uart_close(_uartFrontTmotor);
    uart_close(_uartBackTmotor);
    uart_close(_uartFrontZLWheelhub);
    uart_close(_uartBackZLWheelhub);

    delete _frontTmotorThread;
    delete _backTmotorThread;
    delete _frontZLWheelhubThread;
    delete _backZLWheelhubThread;
    std::cout << "uart stopped! " << std::endl;
}

void HardwareInterface::_eixtMotor()
{
    exitMotorModeCmdTmotor(&_frontModeMsg.TmotorLeftMsg);
    exitMotorModeCmdTmotor(&_frontModeMsg.TmotorRightMsg);

    exitMotorModeCmdTmotor(&_backModeMsg.TmotorLeftMsg);
    exitMotorModeCmdTmotor(&_backModeMsg.TmotorRightMsg);

    exitMotorModeCmdZLWheelhub(&_frontModeMsg.ZLWheelhubMsg);
    exitMotorModeCmdZLWheelhub(&_backModeMsg.ZLWheelhubMsg);

    uartSendTmotor(_uartFrontTmotor, &_frontModeMsg);
    uartSendTmotor(_uartBackTmotor, &_backModeMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);
}

void HardwareInterface::_initDriveTmotor()
{
    enterMotorModeCmdTmotor(&_frontModeMsg.TmotorLeftMsg);
    enterMotorModeCmdTmotor(&_frontModeMsg.TmotorRightMsg);
    uartSendTmotor(_uartFrontTmotor, &_frontModeMsg);

    enterMotorModeCmdTmotor(&_backModeMsg.TmotorLeftMsg);
    enterMotorModeCmdTmotor(&_backModeMsg.TmotorRightMsg);
    uartSendTmotor(_uartBackTmotor, &_backModeMsg);
}

void HardwareInterface::_initDriveZL()
{
    prepareWheelHubDrive1(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive1(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive2(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive2(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive3(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive3(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive4(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive4(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setSynchriniusControl4Wheelhub(&_frontModeMsg.ZLWheelhubMsg);
    setSynchriniusControl4Wheelhub(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setVelocityMode4Wheelhub(&_frontModeMsg.ZLWheelhubMsg);
    setVelocityMode4Wheelhub(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setLeftWheelhubAccTime(&_frontModeMsg.ZLWheelhubMsg);
    setLeftWheelhubAccTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setRightWheelhubAccTime(&_frontModeMsg.ZLWheelhubMsg);
    setRightWheelhubAccTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setLeftWheelhubDecTime(&_frontModeMsg.ZLWheelhubMsg);
    setLeftWheelhubDecTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    setRightWheelhubDecTime(&_frontModeMsg.ZLWheelhubMsg);
    setRightWheelhubDecTime(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive2(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive2(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive3(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive3(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);

    prepareWheelHubDrive4(&_frontModeMsg.ZLWheelhubMsg);
    prepareWheelHubDrive4(&_backModeMsg.ZLWheelhubMsg);
    uartSendZLWheelhub(_uartFrontZLWheelhub, &_frontModeMsg);
    uartSendZLWheelhub(_uartBackZLWheelhub, &_backModeMsg);
}

bool HardwareInterface::sendRecv(RobotCmd *cmd, RobotState *state)
{
    // 0 & 6 front Tmotor; 1 & 7 front ZLWheelhub
    // 2 & 4 back Tmotor; 3 & 5 back ZLWheelhub

    _frontModeControl.TmotorLeftCtrl.kp = cmd->kp[0];
    _frontModeControl.TmotorLeftCtrl.p_des = cmd->q[0];
    _frontModeControl.TmotorLeftCtrl.kd = cmd->kd[0];
    _frontModeControl.TmotorLeftCtrl.v_des = cmd->qd[0];
    _frontModeControl.TmotorLeftCtrl.t_ff = cmd->tau[0];
    // std::cout<<"qd left "<<cmd->qd[0]<<std::endl;

    _frontModeControl.TmotorRightCtrl.kp = cmd->kp[6];
    _frontModeControl.TmotorRightCtrl.p_des = cmd->q[6];
    _frontModeControl.TmotorRightCtrl.kd = cmd->kd[6];
    _frontModeControl.TmotorRightCtrl.v_des = cmd->qd[6];
    _frontModeControl.TmotorRightCtrl.t_ff = cmd->tau[6];
    
    _frontModeControl.ZLWheelhubLeftCtrl.v_des = cmd->qd[1];
    _frontModeControl.ZLWheelhubRightCtrl.v_des = cmd->qd[7];

    _backModeControl.TmotorLeftCtrl.kp = cmd->kp[2];
    _backModeControl.TmotorLeftCtrl.p_des = cmd->q[2];
    _backModeControl.TmotorLeftCtrl.kd = cmd->kd[2];
    _backModeControl.TmotorLeftCtrl.v_des = cmd->qd[2];
    _backModeControl.TmotorLeftCtrl.t_ff = cmd->tau[2];

    _backModeControl.TmotorRightCtrl.kp = cmd->kp[4];
    _backModeControl.TmotorRightCtrl.p_des = cmd->q[4];
    _backModeControl.TmotorRightCtrl.kd = cmd->kd[4];
    _backModeControl.TmotorRightCtrl.v_des = cmd->qd[4];
    _backModeControl.TmotorRightCtrl.t_ff = cmd->tau[4];

    _backModeControl.ZLWheelhubLeftCtrl.v_des = cmd->qd[3];
    _backModeControl.ZLWheelhubRightCtrl.v_des = cmd->qd[5] ;

    if (_zlWheelhubMsgType < 4)
    {
        ++_zlWheelhubMsgType;
    }
    else if (_zlWheelhubMsgType >= 4)
    {
        _zlWheelhubMsgType = 0;
    }

    //  _sendrecv.lock();
    packAllCmdTmotor(&_frontModeMsg, _frontModeControl);
    packAllCmdTmotor(&_backModeMsg, _backModeControl);
    if (_zlWheelhubMsgType == 0)
    {
        packAllCmdZLWheelhub(&_frontModeMsg, _frontModeControl);
        packAllCmdZLWheelhub(&_backModeMsg, _backModeControl);
    }
    else if (_zlWheelhubMsgType == 1)
    {
        readWheelhubVelocity(&_frontModeMsg.ZLWheelhubMsg);
        readWheelhubVelocity(&_backModeMsg.ZLWheelhubMsg);
    }
    else if (_zlWheelhubMsgType == 2)
    {
        readLeftWheelhubPosition(&_frontModeMsg.ZLWheelhubMsg);
        readLeftWheelhubPosition(&_backModeMsg.ZLWheelhubMsg);
    }
    else if (_zlWheelhubMsgType == 3)
    {
        readRightWheelhubPosition(&_frontModeMsg.ZLWheelhubMsg);
        readRightWheelhubPosition(&_backModeMsg.ZLWheelhubMsg);
    }
    _sendrecv.unlock();

    state->q[0] = _frontModeState.TmotorLeftState.p;
    state->qd[0] = _frontModeState.TmotorLeftState.v;
    state->tau[0] = _frontModeState.TmotorLeftState.tau;

    state->q[1] = _frontModeState.ZLWheelhubLeftState.p;
    state->qd[1] = _frontModeState.ZLWheelhubLeftState.v;

    state->q[2] = _backModeState.TmotorLeftState.p;
    state->qd[2] = _backModeState.TmotorLeftState.v;
    state->tau[2] = _backModeState.TmotorLeftState.tau;
    
    state->q[3] = _backModeState.ZLWheelhubLeftState.p;
    state->qd[3] = _backModeState.ZLWheelhubLeftState.v;

    state->q[4] = _backModeState.TmotorRightState.p;
    state->qd[4] = _backModeState.TmotorRightState.v;
    state->tau[4] = _backModeState.TmotorRightState.tau;

    state->q[5] = _backModeState.ZLWheelhubRightState.p;
    state->qd[5] = _backModeState.ZLWheelhubRightState.v;

    state->q[6] = _frontModeState.TmotorRightState.p;
    state->qd[6] = _frontModeState.TmotorRightState.v;
    state->tau[6] = _frontModeState.TmotorRightState.tau;

    state->q[7] = _frontModeState.ZLWheelhubRightState.p;
    state->qd[7] = _frontModeState.ZLWheelhubRightState.v;
    return true;
}


void HardwareInterface::respectiveCommand(float front_left_vel, float front_right_vel, float back_left_vel, float back_right_vel,
                                          float front_left_ang, float front_right_ang, float back_left_ang, float back_right_ang)
{
    // !!!invalid function
    double _maxTurningSpeed = 1;
    cmd.qd[1] = front_left_vel * 60 / 3.14 /0.14;
    cmd.qd[7] = front_right_vel* 60 / 3.14 /0.14;
    cmd.qd[3] = back_left_vel * 60 / 3.14 /0.14;
    cmd.qd[5] = back_right_vel * 60 / 3.14 /0.14;
    
    // 转向电机的位置比例、微分系数
    double kp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double kd[8] = {5, 5, 5, 5, 5, 5, 5, 5};
    //  double kd[8] = {.1, 5, .1, 5, .1, 5, .1, 5};
    for (int i = 0; i < 8; i++)
    {
        cmd.kp[i] = kp[i];
        cmd.kd[i] = kd[i];
    }

    send(&cmd);
}

bool HardwareInterface::send(RobotCmd *cmd)
{
    // cmd->q.reserve(8);
    // cmd->qd.reserve(8);`
    // cmd->tau.reserve(8);
    // cmd->kp.reserve(8);
    // cmd->kd.reserve(8);
    // 0 & 6 front Tmotor; 1 & 7 front ZLWheelhub
    // 2 & 4 back Tmotor; 3 & 5 back ZLWheelhub
    _frontModeControl.TmotorLeftCtrl.kp = cmd->kp[0];
    _frontModeControl.TmotorLeftCtrl.p_des = cmd->q[0];
    _frontModeControl.TmotorLeftCtrl.kd = cmd->kd[0];
    _frontModeControl.TmotorLeftCtrl.v_des = cmd->qd[0];
    _frontModeControl.TmotorLeftCtrl.t_ff = cmd->tau[0];

    _frontModeControl.TmotorRightCtrl.kp = cmd->kp[6];
    _frontModeControl.TmotorRightCtrl.p_des = cmd->q[6];
    _frontModeControl.TmotorRightCtrl.kd = cmd->kd[6];
    _frontModeControl.TmotorRightCtrl.v_des = cmd->qd[6];
    _frontModeControl.TmotorRightCtrl.t_ff = cmd->tau[6];

    _frontModeControl.ZLWheelhubLeftCtrl.v_des = -cmd->qd[1] * 30 /3.14;
    _frontModeControl.ZLWheelhubRightCtrl.v_des = -cmd->qd[7] * 30 /3.14;

    _backModeControl.TmotorLeftCtrl.kp = cmd->kp[2];
    _backModeControl.TmotorLeftCtrl.p_des = cmd->q[2];
    _backModeControl.TmotorLeftCtrl.kd = cmd->kd[2];
    _backModeControl.TmotorLeftCtrl.v_des =cmd->qd[2];
    _backModeControl.TmotorLeftCtrl.t_ff = cmd->tau[2];

    _backModeControl.TmotorRightCtrl.kp = cmd->kp[4];
    _backModeControl.TmotorRightCtrl.p_des = cmd->q[4];
    _backModeControl.TmotorRightCtrl.kd = cmd->kd[4];
    _backModeControl.TmotorRightCtrl.v_des = cmd->qd[4];
    _backModeControl.TmotorRightCtrl.t_ff = cmd->tau[4];
    // std::cout<<"right"<< _backModeControl.TmotorRightCtrl.v_des<<std::endl;

    _backModeControl.ZLWheelhubLeftCtrl.v_des = -cmd->qd[3] * 30 /3.14;
    _backModeControl.ZLWheelhubRightCtrl.v_des = -cmd->qd[5] * 30 /3.14;

    if(_zlWheelhubMsgType < 4){
        ++_zlWheelhubMsgType;
    }
    else if(_zlWheelhubMsgType >= 4){
        _zlWheelhubMsgType = 0;
    }

    //  _sendrecv.lock();
    packAllCmdTmotor(& _frontModeMsg, _frontModeControl);
    packAllCmdTmotor(& _backModeMsg, _backModeControl);
    if(_zlWheelhubMsgType == 0){
        packAllCmdZLWheelhub(& _frontModeMsg, _frontModeControl);
        packAllCmdZLWheelhub(& _backModeMsg, _backModeControl);
    }
    std::vector<float> jointSpeeds;
    for (int i = 0; i < 6; i++)
    {
        jointSpeeds.push_back(cmd->qd[8+i]*180/3.1415);
    }
    _kinovaPtr->ReadAndSetJointSpeeds(jointSpeeds);
    return true;
}

bool HardwareInterface::recv(RobotState *state)
{
    // state->q.reserve(8);
    // state->qd.reserve(8);
    // state->tau.reserve(8);
    // showState(_frontModeState);
    // showState(_backModeState);
    if(_zlWheelhubMsgType == 1){
        readWheelhubVelocity(&_frontModeMsg.ZLWheelhubMsg);
        readWheelhubVelocity(&_backModeMsg.ZLWheelhubMsg);
    }
    else if(_zlWheelhubMsgType == 2){
        readLeftWheelhubPosition(&_frontModeMsg.ZLWheelhubMsg);
        readLeftWheelhubPosition(&_backModeMsg.ZLWheelhubMsg);
    }
    else if(_zlWheelhubMsgType == 3){
        readRightWheelhubPosition(&_frontModeMsg.ZLWheelhubMsg);
        readRightWheelhubPosition(&_backModeMsg.ZLWheelhubMsg);
    }
    _sendrecv.unlock();
    _kinovaPtr->RefreshFeedback();
    std::vector<float> arm_pos = _kinovaPtr->GetJointPositions();
    std::vector<float> arm_vel = _kinovaPtr->GetJointVelocities();
    std::vector<float> arm_tau = _kinovaPtr->GetJointTorques();

    state->q[0] = _frontModeState.TmotorLeftState.p;
    state->qd[0] = _frontModeState.TmotorLeftState.v;
    state->tau[0] = _frontModeState.TmotorLeftState.tau;
    
    state->q[1] = -_frontModeState.ZLWheelhubLeftState.p;
    state->qd[1] = -_frontModeState.ZLWheelhubLeftState.v*3.14 /30;

    state->q[2] = _backModeState.TmotorLeftState.p;
    state->qd[2] = _backModeState.TmotorLeftState.v;
    state->tau[2] = _backModeState.TmotorLeftState.tau;

    state->q[3] = -_backModeState.ZLWheelhubLeftState.p;
    state->qd[3] =- _backModeState.ZLWheelhubLeftState.v* 3.14 /30;

    state->q[4] = _backModeState.TmotorRightState.p;
    state->qd[4] = _backModeState.TmotorRightState.v;
    state->tau[4] = _backModeState.TmotorRightState.tau;

    state->q[5] =- _backModeState.ZLWheelhubRightState.p;
    state->qd[5] =- _backModeState.ZLWheelhubRightState.v * 3.14 /30;

    state->q[6] = _frontModeState.TmotorRightState.p;
    state->qd[6] = _frontModeState.TmotorRightState.v;
    state->tau[6] = _frontModeState.TmotorRightState.tau;

    state->q[7] = -_frontModeState.ZLWheelhubRightState.p;
    state->qd[7] = -_frontModeState.ZLWheelhubRightState.v*3.14 /30;
    for(int i = 0; i < 6; i++){
        state->q[i+8] = arm_pos[i]*3.1415/180;
        state->qd[i+8] = arm_vel[i]*3.1415/180;
        state->tau[i+8] = arm_tau[i];
    }
    return true;
}
}