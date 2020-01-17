/*************************************************
Copyright:wootion
Author: ZhouJinGang
Date:2018-12-25
Description:DeviceNet Master
**************************************************/

#ifndef PROJECT_CDEVICENETMASTER_H
#define PROJECT_CDEVICENETMASTER_H

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <queue>
#include <time.h>
#include <condition_variable>
#include <cmath>
#include <fcntl.h>
#include "controlcan.h"
#include "CDeviceNetCon.h"
#include "SlaveIAIControl.h"
#include "custom_msgs/JoyControlCmd.h"
#include "usb_can.h"

using namespace std;

#define UCMM_DISPLAY_CON_REQ_ID 0x780
#define ONLY_GROUP_TWO_DISPLAY_CON_REQ_ID 0x5FE
#define IO_POLL_SEND_ID 0x5FD
#define IAI_MAC_CHECK_ID 0x5FF
#define IAI_GROUP_TWO_DISPLAY_CON_ID 0x5FB

#define USE_CAN2_RT 0
#define DEBUG_TEST 0

#define CALC_RECEIVE_INTERVAL_TIME 0
#define CAN2_SPECIAL_PRINT 1

enum CONNECTION_TYPE {
    UCMM_REQ = 0, /* UCMM显示信息连接请求 */
    UCMM_RESP = 1, /* UCMM显示信息连接响应 */
    DISPLAY_PRODUCE = 2, /* 显示信息生产者 */
    DISPLAY_CONSUME = 3, /* 显示信息生产者 */
    IO_PRODUCE = 4, /* IO生产者 */
    IO_CONSUME = 5, /* IO消费者 */
    ONLY_GROUP_2_REQ = 6, /* 仅限组2设备连接请求 */
    ONLY_GROUP_2_RESP = 7, /* 仅限组2设备连接响应 */
};
/*CAN的波特率配置
    波特率   定时器0   定时器1  m_unBaud
    5Kbps    0xBF    0xFF    0xFFBF
    10Kbps   0x31    0x1C    0x1C31
    20Kbps   0x18    0x1C    0x1C18
    40Kbps   0x87    0xFF    0xFF87
    50Kbps   0x09    0x1C    0x1C09
    80Kbps   0x83    0Xff    0Xff83
    100Kbps  0x04    0x1C    0x1C04
    125Kbps  0x03    0x1C    0x1C03
    200Kbps  0x81    0xFA    0xFA81
    250Kbps  0x01    0x1C    0x1C01
    400Kbps  0x80    0xFA    0xFA80
    500Kbps  0x00    0x1C    0x1C00
    666Kbps  0x80    0xB6    0xB680
    800Kbps  0x00    0x16    0x1600
    1000Kbps 0x00    0x14    0x1400
*/
enum CAN_BAUD_RATE {
    BAUD_5Kbps = 0xFFBF,
    BAUD_10Kbps = 0x1C31,
    BAUD_20Kbps = 0x1C18,
    BAUD_40Kbps = 0xFF87,
    BAUD_50Kbps = 0x1C09,
    BAUD_80Kbps = 0Xff83,
    BAUD_100Kbps = 0x1C04,
    BAUD_125Kbps = 0x1C03,
    BAUD_200Kbps = 0xFA81,
    BAUD_250Kbps = 0x1C01,
    BAUD_400Kbps = 0xFA80,
    BAUD_500Kbps = 0x1C00,
    BAUD_666Kbps = 0xB680,
    BAUD_800Kbps = 0x1600,
    BAUD_1000Kbps = 0x1400
};

/* 此数据结构存储从站设备初始化参数，调用函数：ZDNMA_AddSlave。 */
typedef struct _tagZDNMA_SLAVECONFIG {
    unsigned dwMacID; /* 从站地址 */
    unsigned dwVendorID; /* 从站厂商ID */
    unsigned dwProductType; /* 产品类型 */
    unsigned dwProductCode; /* 产品代码 */
    unsigned char bScanType; /* I/O连接类型 */
    unsigned char bPollHz; /* 后台轮询使能 */
    unsigned char bStrobeResponseLength; /* 位选通应答长度 */
    unsigned char bStrobeCommandLength; /* 位选通应答长度 */
    unsigned char bPollResponseLength; /* 轮询应答长度 */
    unsigned char bCosCycResponseLength; /* COS/CYC 应答长度 */
    unsigned char bPollCommandLength; /* 轮询命令长度 */
    unsigned char bCosCycCommandLength; /* COS/CYC 命令长度 */
    unsigned dwEprHeartbeat; /* 心跳报文时间(COS/CYC) */
    unsigned dwAckTimer; /* 应答超时时间(COS/CYC) */
    unsigned dwInhibitTimer; /* 报文生产时间(COS/CYC) */
    unsigned dwReserved[4]; /* 保留 */
} DN_SLAVECONFIG, *LPZDNMA_SLAVECONFIG;

typedef struct _tagIAIShaftControl {
    unsigned int shaftNum;
    unsigned int command;
} IAIShaftControl, *PIAIShaftControl;

extern DN_SLAVECONFIG m_slaveConfig;

class CDeviceNetMaster
{
public:
    CDeviceNetMaster();
    ~CDeviceNetMaster();
    bool Init();
    void ReadSlaveConfigFile();
    void RouteReceiveThreadFunc();
    void RouteManageThreadFunc();
    void ManGroupOneMsg(USB_CAN_OBJ *canFrame, int *pollRXSegmentNum);
    void ManGroupTwoMsg(USB_CAN_OBJ *canFrame);
    void ManGroupThreeMsg(USB_CAN_OBJ *canFrame);
    void MacIDCheck();
    void AddSlave();
    void StartScan();
    void UCMM();
    void OnlyGroup2Con();
    uint16 CalcConnectionID(int IDType, u_char msgIg=0);
    int  VerifyCanFrame(USB_CAN_OBJ *canFrame);
    bool CheckUCMMConResp(const USB_CAN_OBJ *canFrame);
    void PollThreadFunction();
    void CheckIAIStatusThreadFunc();
    void SetPollDataPkg(IAIShaftControl *ShaftControl, int ShaftCount = 1);
    void SetIAIIpFixedArea(u_char *pollBuff);
    void SetIAIShaftCommand(uint IAIShaftNUM, uint ctrlCommand, u_char *pollBuff);
    int  CalcSegmentNum(BYTE commandLen);
    void SegmentPollTXFrame(USB_CAN_OBJ *canFrame, u_char *pollBuff, int segmentNum);
    void SegmentPollRXFrame(int *segmentArray, int segmentNum);
    void ManIAIPollResp();
    void IAIShaftsMotion(unsigned unPosNum);
    void JoyCallBack(const custom_msgs::JoyControlCmd::ConstPtr &JoyCommand);

#if USE_CAN2_RT
    void CAN2ThreadFunction();
#endif
    friend void canDataCpy(USB_CAN_OBJ *desCanFrame, USB_CAN_OBJ *souCanFrame);

private:
    VCI_INIT_CONFIG m_config;
    unsigned m_unBaud;
    std::thread *m_pRouteManThread;
    std::thread *m_pRouteRecvThread;
    std::thread *m_pPollThread;
    std::thread *m_pCheckIAIStatusThread;
#if USE_CAN2_RT
    std::thread *m_pCAN2Thread;
#endif
    uint m_unMasterMacID;
    bool m_bMacCheckRet;
    bool m_bIsMasterOnline;
    CDeviceNetCon DeviceNetCon;//多个从站应该要对应多个对象
    USB_CAN_OBJ *m_pollTXFrames;
    USB_CAN_OBJ *m_pollRXFrames;
    u_char *m_pollTXBuff;
    u_char *m_pollRXBuff;
    int m_nSegmentNum;
    int *m_nPollRXSegmentArray;
    std::mutex m_PollTXMutex;
    std::mutex m_PollRXMutex;
    std::mutex m_RouteRXMutex;
    std::vector<USB_CAN_OBJ> m_vCanFrameQueue;
    std::condition_variable m_CANFrameCondition;
    bool m_bIsIAIOnline;
    bool m_bPollFlag;
    bool m_bIsIAIUCMM;
    bool m_bIsIAIOnlyGroupTwo;
    int  m_nIAITimeoutMilliSec;
    int  m_nPollCycleTimeMilliSec;
    timespec m_tIAILastRespTime;
    IAIShaftStatus m_IAIShaftStatus[4];
    IAIStatus m_IAIFixedAreaStatus;
    DN_SLAVECONFIG m_slaveConfig;

    ros::Subscriber m_JoySubscriber;

};

#endif //PROJECT_CDEVICENETMASTER_H
