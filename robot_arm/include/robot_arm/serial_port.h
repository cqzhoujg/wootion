/*
 * serial_port.h
 *
 *  Created on: Nov 22, 2017
 *      Author: LaiHan
 */

#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <string>

namespace SerialPort
{

typedef struct _SerialConfig_
{
    std::string sDevName;
    int nSpeed;
    char chFlowCtrl;
    int nDataBits;
    char chParity;
    int nStopBits;
}SerialConfig;

void PrintBuf(char *pBuf, int nLen, const char *szStartMsg, const char *szEndMsg);

class SerialPort
{
public:
    SerialPort(SerialConfig tConfig = {"/dev/ttyUSB0", 2400, 'N', 8, 'N', 1});
    ~SerialPort();
    void Config(SerialConfig &tConfig);
    bool Open();
    bool Init();
    bool Close();
    void ClearReadBuffer();
    void ClearWriteBuffer();
    int Read(char *szReadBuf, int nDataLen);
    int Write(char *szWriteBuf, int nDataLen);
    int ReadUntil(char *szReadBuf, int nDataLen, int nTimeoutMilliSec);
    int WriteUntil(char *szWriteBuf, int nDataLen, int nTimeoutMilliSec);
private:
    SerialConfig m_tConfig;
    int m_nSerialFd;
    static int m_nSelectTimeoutMilliSec;
    static int m_nOpWaitMilliSec;
};

}

#endif /* _SERIAL_PORT_H_ */
