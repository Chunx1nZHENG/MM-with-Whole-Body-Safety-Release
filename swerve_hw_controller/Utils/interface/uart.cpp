#include "sys/time.h"
#include "uart.h"

using namespace std;

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -25.0f
#define V_MAX 25.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

uint8_t msgFrame[30] = {0x55, 0xaa,                                         // frame head
                        0x1e,                                               // frame length
                        0x03,                                               // cmd
                        0x01, 0x00, 0x00, 0x00,                             // send times
                        0x0a, 0x00, 0x00, 0x00,                             // time lag
                        0x00,                                               // ID type
                        0x00, 0x00, 0x00, 0x00,                             // can ID
                        0x00,                                               // frame type
                        0x08,                                               // can length
                        0x00,                                               // idAcc
                        0x00,                                               // dataAcc
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // can data
                        0x00};                                              // CRC

uint32_t motorID1 = 0x01;
uint32_t motorID2 = 0x02;
uint32_t wheelHubID = 0x0601;

void initMsg(ModeMsg *msg){
    // init msg format
    memcpy(& msg->TmotorLeftMsg, & msgFrame, 30);
    memcpy(& msg->TmotorRightMsg, & msgFrame, 30);
    memcpy(& msg->ZLWheelhubMsg, & msgFrame, 30);

    //set can ID
    for (int i(0); i < 4; ++i){
        msg->TmotorLeftMsg.data[i + 13] = (uint8_t)(motorID1 >> (i * 8));
        msg->TmotorRightMsg.data[i + 13] = (uint8_t)(motorID2 >> (i * 8));
        msg->ZLWheelhubMsg.data[i + 13] = (uint8_t)(wheelHubID >> (i * 8));
    }
}

void enterMotorModeCmdTmotor(JointMsg *msg){
    msg->data[21] = 0xFF;
    msg->data[22] = 0xFF;
    msg->data[23] = 0xFF;
    msg->data[24] = 0xFF;
    msg->data[25] = 0xFF;
    msg->data[26] = 0xFF;
    msg->data[27] = 0xFF;
    msg->data[28] = 0xFC;
}

void exitMotorModeCmdTmotor(JointMsg *msg){
    msg->data[21] = 0xFF;
    msg->data[22] = 0xFF;
    msg->data[23] = 0xFF;
    msg->data[24] = 0xFF;
    msg->data[25] = 0xFF;
    msg->data[26] = 0xFF;
    msg->data[27] = 0xFF;
    msg->data[28] = 0xFD;
}

void prepareWheelHubDrive1(JointMsg *msg){
    msg->data[21] = 0x2B;
    msg->data[22] = 0x40;
    msg->data[23] = 0x60;
    msg->data[24] = 0x00;
    msg->data[25] = 0x00;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void prepareWheelHubDrive2(JointMsg *msg){
    msg->data[21] = 0x2B;
    msg->data[22] = 0x40;
    msg->data[23] = 0x60;
    msg->data[24] = 0x00;
    msg->data[25] = 0x06;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void prepareWheelHubDrive3(JointMsg *msg){
    msg->data[21] = 0x2B;
    msg->data[22] = 0x40;
    msg->data[23] = 0x60;
    msg->data[24] = 0x00;
    msg->data[25] = 0x07;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void prepareWheelHubDrive4(JointMsg *msg){
    msg->data[21] = 0x2B;
    msg->data[22] = 0x40;
    msg->data[23] = 0x60;
    msg->data[24] = 0x00;
    msg->data[25] = 0x0F;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void setSynchriniusControl4Wheelhub(JointMsg *msg){
    msg->data[21] = 0x2B;
    msg->data[22] = 0x0F;
    msg->data[23] = 0x20;
    msg->data[24] = 0x00;
    msg->data[25] = 0x01;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void setVelocityMode4Wheelhub(JointMsg *msg){
    msg->data[21] = 0x2F;
    msg->data[22] = 0x60;
    msg->data[23] = 0x60;
    msg->data[24] = 0x00;
    msg->data[25] = 0x03;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void setLeftWheelhubAccTime(JointMsg *msg){
    msg->data[21] = 0x23;
    msg->data[22] = 0x83;
    msg->data[23] = 0x60;
    msg->data[24] = 0x01;
    msg->data[25] = 0x64;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void setLeftWheelhubDecTime(JointMsg *msg){
    msg->data[21] = 0x23;
    msg->data[22] = 0x83;
    msg->data[23] = 0x60;
    msg->data[24] = 0x02;
    msg->data[25] = 0x64;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void setRightWheelhubAccTime(JointMsg *msg){
    msg->data[21] = 0x23;
    msg->data[22] = 0x84;
    msg->data[23] = 0x60;
    msg->data[24] = 0x01;
    msg->data[25] = 0x64;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void setRightWheelhubDecTime(JointMsg *msg){
    msg->data[21] = 0x23;
    msg->data[22] = 0x84;
    msg->data[23] = 0x60;
    msg->data[24] = 0x02;
    msg->data[25] = 0x64;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void readWheelhubVelocity(JointMsg *msg){
    msg->data[21] = 0x40;
    msg->data[22] = 0x6c;
    msg->data[23] = 0x60;
    msg->data[24] = 0x03;
    msg->data[25] = 0x00;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void readLeftWheelhubPosition(JointMsg *msg){
    msg->data[21] = 0x40;
    msg->data[22] = 0x64;
    msg->data[23] = 0x60;
    msg->data[24] = 0x01;
    msg->data[25] = 0x00;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void readRightWheelhubPosition(JointMsg *msg){
    msg->data[21] = 0x40;
    msg->data[22] = 0x64;
    msg->data[23] = 0x60;
    msg->data[24] = 0x02;
    msg->data[25] = 0x00;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void exitMotorModeCmdZLWheelhub(JointMsg *msg){
    msg->data[21] = 0x2B;
    msg->data[22] = 0x40;
    msg->data[23] = 0x60;
    msg->data[24] = 0x00;
    msg->data[25] = 0x00;
    msg->data[26] = 0x00;
    msg->data[27] = 0x00;
    msg->data[28] = 0x00;
}

void uartSendTmotor(int uartHandle, ModeMsg *msg){
    int waitTime = 100;
    write(uartHandle, & msg->TmotorLeftMsg, 30);
    usleep(waitTime);
    write(uartHandle, & msg->TmotorRightMsg, 30);
    usleep(waitTime);
}

void uartRecvTmotor(int uartHandle, ModeState *state){
    uint8_t replyBuffer[400];
    uint8_t tempBuffer[16];
    int replyLen;

    usleep(1);
    memset(replyBuffer, 0, sizeof(replyBuffer));
    replyLen = read(uartHandle, replyBuffer, sizeof(replyBuffer));

    if ((replyLen > 0) && (replyLen % 16 == 0) && (replyBuffer[0] == 0xaa)){
        int numMsg = replyLen / 16;
        for (int i(0); i < numMsg; ++i){
            for(int j(0); j < 16; ++j){
                tempBuffer[j] = replyBuffer[j+16*i];
            }
            if ((tempBuffer[0] == 0xaa) && (tempBuffer[15] == 0x55)){
                unPackReplyTmotor(tempBuffer, state);
            }
        }
    }
}

void unPackReplyTmotor(uint8_t *msg, ModeState *state){
    uint16_t canID = int(msg[7]);
    uint16_t pInt = (msg[8] << 8) | msg[9];
    uint16_t vInt = (msg[10] << 4) | (msg[11] >> 4);
    uint16_t tInt = ((msg[11] & 0xF) << 8) | msg[12];

    /// convert uints to floats ///
    float p = uintToFloat(pInt, P_MIN, P_MAX, 16);
    float v = uintToFloat(vInt, V_MIN, V_MAX, 12);
    float t = uintToFloat(tInt, -T_MAX, T_MAX, 12);
    
    // uart_mut.lock();

    if(canID == 0x01){
        state->TmotorLeftState.p = p;
        state->TmotorLeftState.v = v;
        state->TmotorLeftState.tau = t;
    }
    else if(canID == 0x02){
        state->TmotorRightState.p = p;
        state->TmotorRightState.v = v;
        state->TmotorRightState.tau = t;
    }

    // uart_mut.unlock();
}

void uartSendRecvTmotor(int uartHandle, ModeMsg *msg, ModeState *state){
    int waitTime = 100;
    double time1 = 0, time2 = 0, time3 = 0;
    // usleep(waitTime);
time1 = recordTime.getMs();
    uartSendTmotor(uartHandle, msg);
time2 = recordTime.getMs();
    uartRecvTmotor(uartHandle, state);
time3 = recordTime.getMs();
// if((time2 - time1) > 1 || (time3 - time2) > 1){
//     std::cout << "send time: " << time2 - time1 << std::endl;
//     std::cout << "recieve time: " << time3 - time2 << std::endl;
// }
}

void packCmdTmotor(JointMsg *msg, JointControl ctrl){
    uint16_t pInt = floatTouint(ctrl.p_des, P_MIN, P_MAX, 16);
    uint16_t vInt = floatTouint(ctrl.v_des, V_MIN, V_MAX, 12);
    uint16_t kpInt = floatTouint(ctrl.kp, KP_MIN, KP_MAX, 12);
    uint16_t kdInt = floatTouint(ctrl.kd, KD_MIN, KD_MAX, 12);
    uint16_t tInt = floatTouint(ctrl.t_ff, T_MIN, T_MAX, 12);
     
    msg->data[21] = pInt >> 8;
    msg->data[22] = pInt & 0xFF;
    msg->data[23] = vInt >> 4;
    msg->data[24] = ((vInt & 0xF) << 4) | (kpInt >> 8);
    msg->data[25] = kpInt & 0xFF;
    msg->data[26] = kdInt >> 4;
    msg->data[27] = ((kdInt & 0xF) << 4) | (tInt >> 8);
    msg->data[28] = tInt & 0xFF;
    // std::cout<<"msg->data[23]"<<vInt<<std::endl;
    // std::cout<<"ctrl.v_des"<<ctrl.v_des<<std::endl;

}

void packAllCmdTmotor(ModeMsg *msg, ModeControl ctrl){
    packCmdTmotor(& (msg->TmotorLeftMsg), ctrl.TmotorLeftCtrl);
    packCmdTmotor(& (msg->TmotorRightMsg), ctrl.TmotorRightCtrl);
}

void uartSendZLWheelhub(int uartHandle, ModeMsg *msg){
    int waitTime = 150;

    write(uartHandle, & msg->ZLWheelhubMsg, 30);

    usleep(waitTime);
}

void uartRecvZLWheelhub(int uartHandle, ModeState *state){
    uint8_t replyBuffer[400];
    uint8_t tempBuffer[16];
    int replyLen;

    usleep(1);
    memset(replyBuffer, 0, sizeof(replyBuffer));
    replyLen = read(uartHandle, replyBuffer, sizeof(replyBuffer));

    if ((replyLen > 0) && (replyLen % 16 == 0) && (replyBuffer[0] == 0xaa)){
        int numMsg = replyLen / 16;
        for (int i(0); i < numMsg; ++i){
            for(int j(0); j < 16; ++j){
                tempBuffer[j] = replyBuffer[j+16*i];
            }
            if ((tempBuffer[0] == 0xaa) && (tempBuffer[15] == 0x55) && (tempBuffer[8] == 0x6C || tempBuffer[8] == 0x64)){
                if(tempBuffer[8] == 0x6c){
                    unPackReplyZLWheelhubVelocity(tempBuffer, state);
                }
                else if(tempBuffer[8] == 0x64){
                    unPackReplyZLWheelhubPosition(tempBuffer, state);
                }
            }
        }
    }
}

void unPackReplyZLWheelhubVelocity(uint8_t *msg, ModeState *state){
    int16_t vIntLeft = (msg[12] << 8) | msg[11];
    int16_t vIntRight = (msg[14] << 8) | msg[13];

    float vLeft = int (vIntLeft) * 0.10;
    float vRight = int (vIntRight) * 0.10;

    // uart_mut.lock();
    state->ZLWheelhubLeftState.v = vLeft;
    state->ZLWheelhubRightState.v = vRight;
    // uart_mut.unlock();
}

void unPackReplyZLWheelhubPosition(uint8_t *msg, ModeState *state){
    int32_t positionCount = (msg[14] << 24 ) | (msg[13] << 16) | (msg[12] << 8) | msg[11];

    // uart_mut.lock();
    if(msg[10] == 0x01){
        state->ZLWheelhubLeftState.p = 3.14 * 2 / 4096 * (int) positionCount;
    }
    else if(msg[10] == 0x02){
        state->ZLWheelhubRightState.p = 3.14 * 2 / 4096 * (int) positionCount;
    }
    // uart_mut.unlock();
}

void uartSendRecvZLWheelhub(int uartHandle, ModeMsg *msg, ModeState *state){
    int waitTime = 100;

    usleep(waitTime);

    uartSendZLWheelhub(uartHandle, msg);
    uartRecvZLWheelhub(uartHandle, state);
}

void packAllCmdZLWheelhub(ModeMsg *msg, ModeControl ctrl){
    uint16_t vIntLeft = (uint16_t) ((int) ctrl.ZLWheelhubLeftCtrl.v_des);
    uint16_t vIntRight = (uint16_t) ((int) ctrl.ZLWheelhubRightCtrl.v_des);
    msg->ZLWheelhubMsg.data[21] = 0x23;
    msg->ZLWheelhubMsg.data[22] = 0xff;
    msg->ZLWheelhubMsg.data[23] = 0x60;
    msg->ZLWheelhubMsg.data[24] = 0x03;
    msg->ZLWheelhubMsg.data[25] = vIntLeft & 0xff;
    msg->ZLWheelhubMsg.data[26] = vIntLeft >> 8;
    msg->ZLWheelhubMsg.data[27] = vIntRight & 0xff;
    msg->ZLWheelhubMsg.data[28] = vIntRight >> 8;
}

float uintToFloat(int xInt, float xMin, float xMax, int bits){
    float span = xMax - xMin;
    float offset = xMin;
    return ((float)xInt) * span / ((float)((1<<bits)-1)) + offset;
}

float floatTouint(float x, float xMin, float xMax, int bits){
    float span = xMax - xMin;
    float offset = xMin;
    return (int) ((x-offset)*((float)((1<<bits)-1)) / span);
}

void showState(ModeState state){
    // cout << "Position of left motor: " << state.TmotorLeftState.p << endl
    cout<< "Velocity of left motor: " << state.TmotorLeftState.v << endl;
    // << "Tau of left motor: " << state.TmotorLeftState.tau << endl
    // << "Position of right motor: " << state.TmotorRightState.p << endl
    cout<< "Velocity of right motor: " << state.TmotorRightState.v << endl;
    // << "Tau of right motor: " << state.TmotorRightState.tau << endl
    // << "Velocity of left wheelhub: " << state.ZLWheelhubLeftState.v << endl
    // << "Position of left wheelhub: " << state.ZLWheelhubLeftState.p << endl
    // << "Velocity of right wheelhub: " << state.ZLWheelhubRightState.v << endl
    // << "Position of right wheelhub: " << state.ZLWheelhubRightState.p << endl;
}

int uart_open(int fd, const char *pathname)
{
    fd = open(pathname, O_RDWR | O_NOCTTY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (-1);
    }
    if (isatty(STDIN_FILENO) == 0)
        printf("standard input is not a terminal device\n");
    return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
        // std::cout << "set in! " << std::endl;

    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    switch (nEvent)
    {
    case 'o':
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'e':
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        break;
    }
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);
    return 0;
}

int uart_init(int baud, char *uart_path)
{
    int fd = uart_open(fd, uart_path);
    if (fd == -1)
    {
        std::cout << "uart error! " << std::endl;
        fprintf(stderr, "uart_open error\n");
        exit(EXIT_FAILURE);
    }
    if (uart_set(fd, baud, 8, 'N', 1) == -1)
    {
        std::cout << "uart failed! " << std::endl;
        fprintf(stderr, "uart set failed!\n");
        exit(EXIT_FAILURE);
    }
    return fd;
}