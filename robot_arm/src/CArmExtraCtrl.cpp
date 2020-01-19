//
// Created by zhoujg on 20-1-19.
//

#include "CArmExtraCtrl.h"


CArmExtraCtrl::CArmExtraCtrl()
{
    m_sSerialPort = "/dev/ttyS0";
    m_nSerialSpeed = 2400;
    m_nActionResponse = 1;
}

bool CArmExtraCtrl::Init(string SerialPort, int nSerialSpeed)
{
    std::string sOutput;

    m_sSerialPort = SerialPort;
    m_nSerialSpeed = nSerialSpeed;

    try
    {
        m_pSerialPort = new SerialPort::SerialPort();
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("malloc serial failed, %s", exception.what());
        return false;
    }

    SerialPort::SerialConfig SerialConfig;
    SerialConfig.sDevName = m_sSerialPort;
    SerialConfig.nSpeed = m_nSerialSpeed;
    SerialConfig.chFlowCtrl = 'n';
    SerialConfig.nDataBits = 8;
    SerialConfig.chParity = 'n';
    SerialConfig.nStopBits = 1;

    m_pSerialPort->Config(SerialConfig);

    if (!m_pSerialPort->Open())
    {
        ROS_ERROR("open serial failed");
        return false;
    }

    if (!m_pSerialPort->Init())
    {
        ROS_ERROR("init serial failed");
        return false;
    }

    if (!SetActionResponse())
    {
        ROS_ERROR("set action response:%d failed", m_nActionResponse);
        return false;
    }

    ROS_INFO("set action response success");
    return true;
}

bool CArmExtraCtrl::SetActionResponse()
{
    char WriteBuf[nWriteFrameSize] = {0};

    if (m_nActionResponse == 1)
    {
        WriteBuf[0] = (char)0xff;
        WriteBuf[1] = (char)0x01;
        WriteBuf[2] = (char)0x00;
        WriteBuf[3] = (char)0x07;
        WriteBuf[4] = (char)0x00;
        WriteBuf[5] = (char)0x61;
        WriteBuf[6] = (char)0x69;
    }
    else
    {
        WriteBuf[0] = (char)0xff;
        WriteBuf[1] = (char)0x01;
        WriteBuf[2] = (char)0x00;
        WriteBuf[3] = (char)0x07;
        WriteBuf[4] = (char)0x00;
        WriteBuf[5] = (char)0x62;
        WriteBuf[6] = (char)0x6a;
    }

    return WriteSerial(WriteBuf, nWriteFrameSize);
}

bool CArmExtraCtrl::ReadSerial(char *szReadBuf, int nDataLen)
{
#ifdef _DEBUG_SERIAL_
    static char szBuf[nWriteFrameSize * 3 + 4] = {0};
    char *pChar;
    int i;
    timespec ts1, ts2;
#endif
    int nRet;

#ifdef _DEBUG_SERIAL_
    timespec_get(&ts1, TIME_UTC);
#endif

    nRet = m_pSerialPort->ReadUntil(szReadBuf, nDataLen, nMaxOpTimeMilliSec);
    if (nRet != nDataLen)
    {
        ROS_ERROR("only read %d of %d bytes", nRet, nDataLen);
        return false;
    }

#ifdef _DEBUG_SERIAL_
    timespec_get(&ts2, TIME_UTC);
    pChar = szBuf;
    memset(szBuf, 0, nWriteFrameSize * 3 + 4);
    sprintf(pChar, "[");
    pChar++;
    for (i = 0; i < nRet; i++)
    {
        sprintf(pChar, " %02x ", (unsigned char)szReadBuf[i]);
        pChar += 3;
    }
    sprintf(pChar, " ]");

    ROS_INFO("read %dbytes(%lums): %s", nRet, ((ts2.tv_sec-ts1.tv_sec)*1000000000+(ts2.tv_nsec-ts1.tv_nsec))/1000000, szBuf);
#endif

    return true;
}

bool CArmExtraCtrl::WriteSerial(char *szWriteBuf, int nDataLen)
{
#ifdef _DEBUG_SERIAL_
    static char szBuf[nWriteFrameSize * 3 + 4] = {0};
    char *pChar;
    int i;
    timespec ts1, ts2;
#endif
    int nRet;

#ifdef _DEBUG_SERIAL_
    timespec_get(&ts1, TIME_UTC);
#endif

    m_pSerialPort->ClearWriteBuffer();
    nRet = m_pSerialPort->Write(szWriteBuf, nDataLen);
    if (nRet != nDataLen)
    {
        ROS_ERROR("only write %d of %d bytes", nRet, nDataLen);
        return false;
    }

#ifdef _DEBUG_SERIAL_
    timespec_get(&ts2, TIME_UTC);
    pChar = szBuf;
    memset(szBuf, 0, nWriteFrameSize * 3 + 4);
    sprintf(pChar, "[");
    pChar++;
    for (i = 0; i < nRet; i++)
    {
        sprintf(pChar, " %02x", (unsigned char)szWriteBuf[i]);
        pChar += 3;
    }
    sprintf(pChar, " ]");

    ROS_INFO("write %dbytes(%lums): %s", nRet, ((ts2.tv_sec-ts1.tv_sec)*1000000000+(ts2.tv_nsec-ts1.tv_nsec))/1000000, szBuf);
#endif

    return true;
}

bool CArmExtraCtrl::ExtractFrameData(char *pBuf, FrameData *pFrameData)
{
    DataUnion Data = {0};

    pFrameData->Data = 0;
    pFrameData->Type = frame_unknown;

    if (pBuf[0] == (char)0xff && pBuf[1] == (char)0x01 && pBuf[2] == (char)0x00)
    {
        if (calcChecksum(pBuf) == pBuf[6])
        {
            if (pBuf[3] == (char)0x59)
            {
                Data.Byte[0] = pBuf[5];
                Data.Byte[1] = pBuf[4];
                pFrameData->Type = pan_angle_ack;

                if (Data.Num == 36000)
                {
                    Data.Num = 0;
                }

                pFrameData->Data = Data.Num;
                return true;
            }
            else if (pBuf[3] == (char)0x5b)
            {
                Data.Byte[0] = pBuf[5];
                Data.Byte[1] = pBuf[4];
                pFrameData->Type = tilt_angle_ack;

                if (Data.Num == 36000)
                {
                    Data.Num = 0;
                }

                pFrameData->Data = Data.Num;
                return true;
            }
        }
        else
        {
            ROS_WARN("frame check sum error");
        }
    }

    return false;
}

char CArmExtraCtrl::calcChecksum(char *buf)
{
    int i;
    char sum = 0;

    for (i = 1; i < 6; ++i)
    {
        sum += buf[i];
    }

    return (char)(sum % 0x100);
}

bool CArmExtraCtrl::ExecuteTask(int nCmd)
{
    char WriteBuf[nWriteFrameSize];
    switch(nCmd)
    {
        case LIGHT_ON:
            WriteBuf[0] = (char)0xff;
            WriteBuf[1] = (char)0x01;
            WriteBuf[2] = (char)0x00;
            WriteBuf[3] = (char)0x09;
            WriteBuf[4] = (char)0x00;
            WriteBuf[5] = (char)0x02;
            WriteBuf[6] = calcChecksum(WriteBuf);
            break;
        case LIGHT_OFF:
            WriteBuf[0] = (char)0xff;
            WriteBuf[1] = (char)0x01;
            WriteBuf[2] = (char)0x00;
            WriteBuf[3] = (char)0x0b;
            WriteBuf[4] = (char)0x00;
            WriteBuf[5] = (char)0x02;
            WriteBuf[6] = calcChecksum(WriteBuf);
            break;
        case BURSH_ON:
            WriteBuf[0] = (char)0xff;
            WriteBuf[1] = (char)0x01;
            WriteBuf[2] = (char)0x00;
            WriteBuf[3] = (char)0x09;
            WriteBuf[4] = (char)0x00;
            WriteBuf[5] = (char)0x01;
            WriteBuf[6] = calcChecksum(WriteBuf);
            break;
        case BURSH_OFF:
            WriteBuf[0] = (char)0xff;
            WriteBuf[1] = (char)0x01;
            WriteBuf[2] = (char)0x00;
            WriteBuf[3] = (char)0x0b;
            WriteBuf[4] = (char)0x00;
            WriteBuf[5] = (char)0x01;
            WriteBuf[6] = calcChecksum(WriteBuf);
            break;
        default:
            ROS_ERROR("[ExecuteTask] cmd error, nCmd=%d",nCmd);
            return false;
    }

    if (!WriteSerial(WriteBuf, nWriteFrameSize))
    {
        ROS_ERROR("[ExecuteTask] write serial failed");
        return false;
    }

    return true;
}

CArmExtraCtrl::~CArmExtraCtrl()
{
    delete m_pSerialPort;
}