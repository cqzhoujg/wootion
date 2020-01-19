//
// Created by zhoujg on 20-1-19.
//

#ifndef PROJECT_CARMEXTRACTRL_H
#define PROJECT_CARMEXTRACTRL_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "serial_port.h"

using namespace std;

static const int nWriteFrameSize = 7;
static const int nReadFrameSize = 7;
static const int nMaxOpTimeMilliSec = 15000;

typedef enum _tagFrameType_
{
    pan_angle_ack,
    tilt_angle_ack,
    frame_unknown
}FrameType;

typedef enum _tagTaskCmd_
{
    LIGHT_ON,
    LIGHT_OFF,
    BURSH_ON,
    BURSH_OFF
}TaskCmd;

typedef struct _tagFrameData_
{
    FrameType Type;
    int       Data;
}FrameData;

typedef union _tagDataUnion_
{
    int  Num;
    char Byte[sizeof(int)];
}DataUnion;

class CArmExtraCtrl
{
public:
    CArmExtraCtrl();
    ~CArmExtraCtrl();

    bool Init(string SerialPort, int nSerialSpeed);
    bool SetActionResponse();
    bool ReadSerial(char *szReadBuf, int nDataLen);
    bool WriteSerial(char *szWriteBuf, int nDataLen);
    bool ExtractFrameData(char *pBuf, FrameData *pFrameData);
    bool ExecuteTask(int nCmd);
    char calcChecksum(char *buf);

private:
    string m_sSerialPort;

    int m_nSerialSpeed;
    int m_nActionResponse;

    char m_szReadBuf[nReadFrameSize];
    FrameData m_ReadFrame;

    SerialPort::SerialPort *m_pSerialPort;

};

#endif //PROJECT_CARMEXTRACTRL_H
