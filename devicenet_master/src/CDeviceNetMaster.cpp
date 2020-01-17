/*************************************************
Copyright:wootion
Author: ZhouJinGang
Date:2018-12-25
Description:DeviceNet Master
**************************************************/

#include "CDeviceNetMaster.h"
/*************************************************
Function: CDeviceNetMaster::CDeviceNetMaster
Description:　CDeviceNetMaster类构造函数
Input: void
Output: void
Others: void
**************************************************/
CDeviceNetMaster::CDeviceNetMaster():
    m_unBaud(BAUD_125Kbps),
    m_unMasterMacID(0x00),
    m_bMacCheckRet(true),
    m_bIsMasterOnline(false),
    m_bIsIAIOnline(false),
    m_bPollFlag(false),
    m_bIsIAIUCMM(false),
    m_bIsIAIOnlyGroupTwo(false),
    m_nIAITimeoutMilliSec(1000),
    m_nPollCycleTimeMilliSec(20)
{
    ros::NodeHandle PublicNodeHandle;
    ros::NodeHandle PrivateNodeHandle("~");

    m_tIAILastRespTime.tv_sec = 0;
    m_tIAILastRespTime.tv_nsec = 0;

    if(!Init())
    {
        exit(-1);
    }

#if USE_CAN2_RT//程序测试线程
    try
    {
        m_pCAN2Thread = new std::thread(std::bind(&CDeviceNetMaster::CAN2ThreadFunction, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CDeviceNetMaster::CDeviceNetMaster]malloc CAN2 thread failed, %s", exception.what());
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }
#endif

    try
    {
        m_pRouteRecvThread = new std::thread(std::bind(&CDeviceNetMaster::RouteReceiveThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CDeviceNetMaster::CDeviceNetMaster]malloc route receive thread failed, %s", exception.what());
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }

    try
    {
        m_pRouteManThread = new std::thread(std::bind(&CDeviceNetMaster::RouteManageThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CDeviceNetMaster::CDeviceNetMaster]malloc route manage thread failed, %s", exception.what());
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }

//加延时 1s 或者标志位 以确保以上线程起来后在继续执行。
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    MacIDCheck();

    AddSlave();

    StartScan();

    if(m_bIsIAIUCMM)
    {
        UCMM();
    }
    else if(m_bIsIAIOnlyGroupTwo)
    {
        OnlyGroup2Con();
    }

    try
    {
        m_pPollThread = new std::thread(std::bind(&CDeviceNetMaster::PollThreadFunction, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CDeviceNetMaster::CDeviceNetMaster]malloc poll thread failed, %s", exception.what());
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }

    try
    {
        m_pCheckIAIStatusThread = new std::thread(std::bind(&CDeviceNetMaster::CheckIAIStatusThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("malloc Check IAI Status thread failed, %s", exception.what());
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }

    m_JoySubscriber = PublicNodeHandle.subscribe<custom_msgs::JoyControlCmd>("joy_cmd", 10, boost::bind(&CDeviceNetMaster::JoyCallBack, this, _1));

    ROS_INFO("[CDeviceNetMaster][CDeviceNetMaster] end...");
}

/*************************************************
Function: CDeviceNetMaster::Init
Description:　初始化　UsbCan　设备
Input: void
Output: true  初始化成功
        false 初始化失败
Others: void
**************************************************/
bool CDeviceNetMaster::Init()
{
    ROS_INFO("[CDeviceNetMaster][Init] begin...");
    DWORD ret = 0;
    if((ret=VCI_OpenDevice(VCI_USBCAN2,0,0))==1)//打开设备
    {
        ROS_INFO("[CDeviceNetMaster][Init] open deivce VCI_USBCAN2 success!");//打开设备成功
    }
    else
    {
        ROS_INFO("[CDeviceNetMaster][Init] VCI_OpenDevice ret = %d",ret);
        ROS_INFO("[CDeviceNetMaster][Init] open device VCI_USBCAN2 error!");
        exit(1);
    }

    m_config.AccCode = 0;
    m_config.AccMask = 0xffffffff;
    m_config.Filter = 1;
    m_config.Mode = 0;
    m_config.Timing0 = UCHAR(m_unBaud & 0xff);
    m_config.Timing1 = UCHAR(m_unBaud >> 8);

    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &m_config) != 1)
    {
        ROS_INFO("[CDeviceNetMaster][Init] VCI_InitCAN1 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_INFO("[CDeviceNetMaster][Init] VCI_InitCAN1 succeeded");

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        ROS_INFO("[CDeviceNetMaster][Init] VCI_StartCAN1 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_INFO("[CDeviceNetMaster][Init] VCI_StartCAN1 succeeded");

#if USE_CAN2_RT
    if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &m_config) != 1)
    {
        ROS_INFO("[CDeviceNetMaster][Init] VCI_InitCAN2 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_INFO("[CDeviceNetMaster][Init] VCI_InitCAN2 succeeded");

    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
    {
        ROS_INFO("[CDeviceNetMaster][Init] VCI_StartCAN2 failed");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    ROS_INFO("[CDeviceNetMaster][Init] VCI_StartCAN2 succeeded");
#endif

#if CALC_RECEIVE_INTERVAL_TIME
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    USB_CAN_OBJ ReqFrame;

    setCanFrame(&ReqFrame, UCMM_DISPLAY_CON_REQ_ID, 4, 0x3F4B0234);

    VCI_CAN_OBJ can[3000];
    int recLen = 0;
    timespec ts1, ts2;//结构体 两个成员：秒、纳秒;是自1970年1月1号00:00:00到现在的秒数
    long pollInterval;

    for (int i = 0; i < 3; i++)
    {
        usbCan_Transmit(&ReqFrame, 1);
        printCanFrame(&ReqFrame, "[CDeviceNetMaster][Init]");
        timespec_get(&ts1, TIME_UTC);
        if((recLen=VCI_Receive(VCI_USBCAN2, 0, 0, can, 3000, 100/*ms*/))>0)
        {
            timespec_get(&ts2, TIME_UTC);
            for(int ii=0;ii<recLen;ii++)
            {
                PrintCanFrame(&can[ii], "[CDeviceNetMaster][Init]");
            }
            ROS_INFO("[CDeviceNetMaster][Init] ts1.tv_sec=%ld,ts1.tv_nsec=%ld",ts1.tv_sec,ts1.tv_nsec);
            ROS_INFO("[CDeviceNetMaster][Init] ts2.tv_sec=%ld,ts2.tv_nsec=%ld",ts2.tv_sec,ts2.tv_nsec);
            ROS_INFO("[CDeviceNetMaster][Init] sec=%ld,nsec=%fms",(ts2.tv_sec-ts1.tv_sec),(ts2.tv_nsec-ts1.tv_nsec)/1000000.0);
        }
    }
    VCI_CloseDevice(VCI_USBCAN2,0);
    exit(-1);
#endif

    ROS_INFO("[CDeviceNetMaster][Init] end...");
    return true;
}
/*************************************************
Function: RouteReceiveThreadFunc::Init
Description:　 USBCAN 设备数据接收线程,用于DN总线上接收到的所有数据的接收
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::RouteReceiveThreadFunc()
{
    int i,recLen;
    USB_CAN_OBJ rec[3000];
    while (ros::ok())
    {
//        if((recLen = usbCan_Receive(rec, 3000) > 0))//这里如果对接受函数进行二次封装调用的话，会出现丢帧的问题，原因不明。
        if((recLen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100/*ms*/))>0)
        {
            std::unique_lock<std::mutex> lock(m_RouteRXMutex);
            for(i=0; i<recLen; i++)
            {
//                PrintCanFrame(&rec[i], "[CDeviceNetMaster][RouteReceiveThreadFunc]");
                m_vCanFrameQueue.push_back(rec[i]);
            }
//            ROS_INFO("[CDeviceNetMaster][RouteReceiveThreadFunc] m_dCanRXDeque size=%ld", m_dCanRXDeque.size());
            lock.unlock();
            m_CANFrameCondition.notify_one();
        }
    }
}

/*************************************************
Function: CDeviceNetMaster::RouteManageThreadFunc
Description: USBCAN设备数据处理线程,用于DN总线上接收到的所有数据的分发或处理(消费者)
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::RouteManageThreadFunc()
{
    USB_CAN_OBJ canTemp;
    std::vector<USB_CAN_OBJ> vCanFrameQueue;
    unsigned long ulSize;
    int pollRXSegmentNum = 0;

    ROS_INFO("[CDeviceNetMaster][RouteManageThreadFunc] begin...");
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_RouteRXMutex);
        while(m_vCanFrameQueue.empty())
        {
            m_CANFrameCondition.wait(lock);
        }

        vCanFrameQueue.assign(m_vCanFrameQueue.begin(), m_vCanFrameQueue.end());
        m_vCanFrameQueue.clear();
        lock.unlock();

        ulSize = vCanFrameQueue.size();
//        ROS_INFO("m_dCanRXDeque ulSize = %ld.",ulSize);
        for (int i = 0; i < ulSize; i++)
        {
            canTemp = vCanFrameQueue[i];
            if (!VerifyCanFrame(&canTemp))
            {
                ROS_INFO("[CDeviceNetMaster][RouteManageThreadFunc] verify frame failed");
                break;
            }
//            PrintCanFrame(&canTemp, "[CDeviceNetMaster][RouteManageThreadFunc]);
            if (canTemp.ID <= 0x3ff)//组1
            {
                ManGroupOneMsg(&canTemp, &pollRXSegmentNum);
            }
            else if (canTemp.ID <= 0x5ff)//组2
            {
                ManGroupTwoMsg(&canTemp);
            }
            else if (canTemp.ID <= 0x7bf)//组3
            {
                ManGroupThreeMsg(&canTemp);
            }
            else if (canTemp.ID <= 0x7ef)//组4
            {
                ROS_INFO("[CDeviceNetMaster][RouteManageThreadFunc] group4 message");
            }
            else//无效信息
            {
                ROS_INFO("[CDeviceNetMaster][RouteManageThreadFunc] invalid message");
                printCanFrame(&canTemp, "[CDeviceNetMaster][RouteManageThreadFunc]");
            }
        }

    }
    vCanFrameQueue.clear();
}

/*************************************************
Function: CDeviceNetMaster::ManGroupOneMsg
Description: 处理及分发所有can总线上接收到的组1报文
Input: USB_CAN_OBJ *canFrame 接受到的CAN帧数据
       int *pollRXSegmentNum 分段协议的分段计数,用于监测是否有数据丢帧
Output: void
Others: void
**************************************************/
/**/
void CDeviceNetMaster::ManGroupOneMsg(USB_CAN_OBJ *canFrame, int *pollRXSegmentNum)
{
    unsigned int macID = 0, msgID = 0;
//    PrintCanFrame(canFrame, "[CDeviceNetMaster][ManGroupOneMsg]");
    macID = canFrame->ID & 0x3f;
    msgID = (canFrame->ID >> 6) & 0x0F;

    int *SegmentNum = pollRXSegmentNum;
    bool pollRXSegmentErr = false;

//    ROS_INFO("[CDeviceNetMaster][RouteReceiveThreadFunc] macID=%d,msgID=%d",macID,msgID);
    if(macID == m_slaveConfig.dwMacID)//所有来自IAI从站的组1消息处理
    {
        if (msgID == 0x0f)//轮询响应报文(轮询响应或状态变化/循环应答信息；DN协议规定的固定信息ID为0xF)
        {
            for(int i=0;i<m_nSegmentNum;i++)
            {
                if(canFrame->Data[0] == m_nPollRXSegmentArray[i])
                {
                    if(*SegmentNum != i)
                    {
                        ROS_INFO("[CDeviceNetMaster][ManGroupOneMsg] poll received data fragmentation error %d",i);
                        pollRXSegmentErr = true;
                        break;
                    }
                    canDataCpy(&m_pollRXFrames[i], canFrame);
                    (*SegmentNum)++;
                }
            }

            if (*SegmentNum == m_nSegmentNum && !pollRXSegmentErr)//只取完整的一组控制响应数据
            {
                std::unique_lock<std::mutex> lock(m_PollRXMutex);
                int pollRXBufPos = 0;

                for (int i = 0; i < m_nSegmentNum; i++)
                {
                    for (int j = 1; j < m_pollRXFrames[i].DataLen; j++)
                    {
                        m_pollRXBuff[pollRXBufPos] = m_pollRXFrames[i].Data[j];
                        pollRXBufPos++;
                    }
                }
                lock.unlock();
                *SegmentNum = 0;
            }
            if (*SegmentNum == m_nSegmentNum || pollRXSegmentErr)
            {
                *SegmentNum = 0;
            }


            if (m_bPollFlag)//开始轮询后监测IAI的在线情况
            {
                timespec_get(&m_tIAILastRespTime, TIME_UTC);
    //            ROS_INFO("[CDeviceNetMaster][ManGroupOneMsg] m_tIAILastRespTime.tv_sec=%ld,m_tIAILastRespTime.tv_nsec=%ld",m_tIAILastRespTime.tv_sec,m_tIAILastRespTime.tv_nsec);
            }
        }
    }
}

/*************************************************
Function: CDeviceNetMaster::ManGroupTwoMsg
Description: 处理及分发所有can总线上接收到的组2报文
Input: USB_CAN_OBJ *canFrame 接受到的CAN帧数据
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::ManGroupTwoMsg(USB_CAN_OBJ *canFrame)
{
    unsigned int macID = 0, msgID = 0;
    macID = (canFrame->ID >> 3) & 0x3f;
    msgID = canFrame->ID & 0x07;

    USB_CAN_OBJ *canTemp = canFrame;
    if (canTemp->ID == 0x407)//主站macid ‘0’已被占用
    {
        if(!checkCanFrame(canTemp, 0))
            m_bMacCheckRet = false;
        if (m_bIsMasterOnline)
        {
            USB_CAN_OBJ checkReqFrame;
            checkReqFrame.ID = 0x407;//主站的mac默认为0
            checkReqFrame.RemoteFlag = 0;
            checkReqFrame.ExternFlag = 0;
            checkReqFrame.DataLen = 7;
            checkReqFrame.Data[0] = 0x00;
            //制造商ID
            checkReqFrame.Data[1] = 0x00;
            checkReqFrame.Data[2] = 0x00;
            //系列号
            checkReqFrame.Data[3] = 0x00;
            checkReqFrame.Data[4] = 0x00;
            checkReqFrame.Data[5] = 0x00;
            checkReqFrame.Data[6] = 0x00;

            usbCan_Transmit(&checkReqFrame, 1);
        }
    }

    if(macID == m_slaveConfig.dwMacID)//所有来自IAI从站的组3消息处理。
    {
        if(msgID == 7)//IAI_MAC_CHECK_ID 0x5ff, msgID=7是DN协议规定的mac检测msgID
        {
            m_bIsIAIOnline = true;
        }
        if(msgID == 3)//IAI_GROUP_TWO_DISPLAY_CON_ID 0x5FB, msgID=3是DN协议规定的从站显示/非连接响应信息。
        {
            m_bIsIAIOnline = true;
            m_bIsIAIOnlyGroupTwo = true;
        }
    }
}

/*************************************************
Function: CDeviceNetMaster::ManGroupThreeMsg
Description: 处理及分发所有can总线上接收到的组3报文
Input: USB_CAN_OBJ *canFrame 接受到的CAN帧数据
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::ManGroupThreeMsg(USB_CAN_OBJ *canFrame)
{
    USB_CAN_OBJ *canTemp = canFrame;
    unsigned int macID = 0, msgID = 0;
    macID = canTemp->ID & 0x3f;
    msgID = (canTemp->ID >> 6) & 0x07;
//    ROS_INFO("[CDeviceNetMaster][ManGroupThreeMsg] macID=%d,msgID=%d",macID,msgID);
    if (macID == m_slaveConfig.dwMacID)//所有来自IAI从站的组3消息处理。
    {
        if (msgID == 6)//非连接请求信息
        {

        }
        if (msgID == 5)//非连接响应信息
        {
            if(!m_bIsIAIUCMM)
            {
                m_bIsIAIUCMM = true;
                m_bIsIAIOnline = true;
                return;
            }

            ROS_INFO("[CDeviceNetMaster][ManGroupThreeMsg] msgID is 5, awake ReceiveMutex...");
            std::unique_lock<std::mutex> lock(ReceiveMutex);

            USB_CAN_OBJ can;
            canDataCpy(&can, canFrame);
            shareCanData.push_back(can);

            lock.unlock();
            ReceiveCondition.notify_one();
        }
        if(msgID == 4)//UCMM响应后，基于显示信息连接的交互报文处理( 与UCMM()中的”unsigned unSourceID = 0x04;//源信息ID为4“对应)
        {
            ROS_INFO("[CDeviceNetMaster][ManGroupThreeMsg] msgID is 4, awake ReceiveMutex...");
            printCanFrame(canTemp, "[CDeviceNetMaster][ManGroupThreeMsg]");
            std::unique_lock<std::mutex> lock(ReceiveMutex);

            USB_CAN_OBJ can;
            canDataCpy(&can, canFrame);
            shareCanData.push_back(can);

            lock.unlock();
            ReceiveCondition.notify_one();
        }
    }

}

#if USE_CAN2_RT
/*************************************************
Function: CDeviceNetMaster::CAN2ThreadFunction
Description: 用于测试，实现can总线报文的捕获，由于can分析仪的usb口只能被一个进程中open，导致两路can只能被单独的进程使用,
             所以这里只能单独起个线程
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::CAN2ThreadFunction()
{
    ROS_INFO("[CDeviceNetMaster][CAN2ThreadFunction] is running...");
    int recLen=0;
    USB_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i,j;
    DWORD ind=1;
    int count=0, frameCount=0;
    int testOffLine = 1;
    int testLostFrame = 1;

    while(ros::ok())
    {
        if((recLen=VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100))>0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            for(j=0;j<recLen;j++)
            {
                printf("[CDeviceNetMaster][CAN2ThreadFunction] ");
                printf("Index:%04d  ",count);count++;//序号递增
                printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
                if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                printf("DLC:0x%02X",rec[j].DataLen);//帧长度
                printf(" data:0x");	//数据
                for(i = 0; i < rec[j].DataLen; i++)
                {
                    printf(" %02X", rec[j].Data[i]);
                }
//                printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
                printf("\n");

#if(DEBUG_TEST)
                if(rec[j].ID == UCMM_DISPLAY_CON_REQ_ID)
                {
                    ROS_INFO("[CDeviceNetMaster][CAN2ThreadFunction] send can frame:0x3F4B0234");
                    USB_CAN_OBJ UCMMReqFrame;
                    setCanFrame(&UCMMReqFrame, 0x77f, 6, 0x00cb02040a00);
                    VCI_Transmit(VCI_USBCAN2, 0, 1, &UCMMReqFrame, 1);
                }

                if(rec[j].ID == 0x700)
                {
                    ROS_INFO("[CDeviceNetMaster][CAN2ThreadFunction] send can frame:0x001122334455");
                    USB_CAN_OBJ UCMMReqFrame;
                    setCanFrame(&UCMMReqFrame, 0x73f, 6, 0x001122334455);
                    VCI_Transmit(VCI_USBCAN2, 0, 1, &UCMMReqFrame, 1);

                    if(checkCanFrame(&rec[j], 0xBF814B00))
                    {//Index:0010  CAN2 RX ID:0x00000700 Standard  Data   DLC:0x04 data:0x BF 81 4B 00
                        setCanFrame(&UCMMReqFrame, 0x73f, 6, 0x554433221100);
                        VCI_Transmit(VCI_USBCAN2, 0, 1, &UCMMReqFrame, 1);
                    }
                }

/*
Index:0421  CAN2 RX ID:0x000003FF Standard  Data   DLC:0x08 data:0x 00 00 80 0F 00 00 00 00
Index:0422  CAN2 RX ID:0x000003FF Standard  Data   DLC:0x08 data:0x 41 00 00 00 00 00 00 00
Index:0423  CAN2 RX ID:0x000003FF Standard  Data   DLC:0x08 data:0x 42 00 00 00 00 02 70 00
Index:0424  CAN2 RX ID:0x000003FF Standard  Data   DLC:0x08 data:0x 43 00 02 70 00 00 02 70
Index:0425  CAN2 RX ID:0x000003FF Standard  Data   DLC:0x05 data:0x 84 00 00 02 70
*/
                if(rec[j].ID == 0x5fd && testOffLine)
                {
                    frameCount++;
                    if(frameCount == 5)
                    {
                        ROS_INFO("[CDeviceNetMaster][CAN2ThreadFunction] send poll can frame--");
                        USB_CAN_OBJ UCMMReqFrame[5];
                        setCanFrame(&UCMMReqFrame[0], 0x3ff, 8, 0x0000800F00000000);
                        setCanFrame(&UCMMReqFrame[1], 0x3ff, 8, 0x4100000000000000);
                        setCanFrame(&UCMMReqFrame[2], 0x3ff, 8, 0x4200000000027000);
                        setCanFrame(&UCMMReqFrame[3], 0x3ff, 8, 0x4300027000000270);
                        if(testLostFrame != 1)
                        {
                            setCanFrame(&UCMMReqFrame[4], 0x3ff, 5, 0x8400000270);
                            VCI_Transmit(VCI_USBCAN2, 0, ind, UCMMReqFrame, 5);
                        }
                        else
                        {
                            ROS_INFO("[CDeviceNetMaster][CAN2ThreadFunction] only send 4 can frames--");
                            VCI_Transmit(VCI_USBCAN2, 0, ind, UCMMReqFrame, 4);
                        }
                        frameCount = 0;
                        testLostFrame = 2;
//                        testOffLine = 0;
                    }
                }
#endif
            }
        }
//        ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。
    }
    printf("run thread exit\n");//退出接收线程
}
#endif

/*************************************************
Function: CDeviceNetMaster::MacIDCheck
Description: 主站MACID检测
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::MacIDCheck()
{
    ROS_INFO("[CDeviceNetMaster][MacIDCheck] begin...");
    USB_CAN_OBJ checkReqFrame;

    checkReqFrame.ID = 0x407;//主站的mac默认为0
    checkReqFrame.RemoteFlag = 0;
    checkReqFrame.ExternFlag = 0;
    checkReqFrame.DataLen = 7;
    checkReqFrame.Data[0] = 0x00;
    //制造商ID
    checkReqFrame.Data[1] = 0x00;
    checkReqFrame.Data[2] = 0x00;
    //系列号
    checkReqFrame.Data[3] = 0x00;
    checkReqFrame.Data[4] = 0x00;
    checkReqFrame.Data[5] = 0x00;
    checkReqFrame.Data[6] = 0x00;

    usbCan_Transmit(&checkReqFrame, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if(!m_bMacCheckRet)
    {
        ROS_INFO("[CDeviceNetMaster][MacIDCheck] macID [0] is used by other device.");
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }
    usbCan_Transmit(&checkReqFrame, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if(!m_bMacCheckRet)
    {
        ROS_INFO("[CDeviceNetMaster][MacIDCheck] macID [0] is used by other device.");
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }
    m_bIsMasterOnline = true;
    ROS_INFO("[CDeviceNetMaster][MacIDCheck] DeviceMaster is online.");

    ROS_INFO("[CDeviceNetMaster][macIDCheck] end");
}

/*************************************************
Function: CDeviceNetMaster::AddSlave
Description: 添加从站
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::AddSlave()
{
    ROS_INFO("[CDeviceNetMaster][AddSlave] begin...");
    ReadSlaveConfigFile();//读写eds文件 还需要在定义个config结构体
    //这里后续可以通过读取文件获取
    m_slaveConfig.dwMacID = 0x3f;
    m_slaveConfig.dwVendorID = 699;
    m_slaveConfig.dwProductType = 0;
    m_slaveConfig.dwProductCode = 106;
    m_slaveConfig.bScanType = 1;//Default IO Connection = Poll 轮询
    m_slaveConfig.bPollHz = 1;
    m_slaveConfig.bPollResponseLength = 32;//定位器2控制模式，有效自己数为32，加上分段表示等工37个字节 4*(1+7)+(1+4)
    m_slaveConfig.bPollCommandLength = 32;//定位器2控制模式，有效自己数为32，加上分段表示等工37个字节

    m_nSegmentNum = CalcSegmentNum(m_slaveConfig.bPollCommandLength);

    if (m_nSegmentNum > 0)
    {
        m_pollTXFrames = new USB_CAN_OBJ[m_nSegmentNum];
        m_pollRXFrames = new USB_CAN_OBJ[m_nSegmentNum];

        m_nPollRXSegmentArray = new int[m_nSegmentNum];
        SegmentPollRXFrame(m_nPollRXSegmentArray, m_nSegmentNum);

        m_pollTXBuff = new u_char[m_slaveConfig.bPollCommandLength];
        memset(m_pollTXBuff,0x00,m_slaveConfig.bPollCommandLength);

        m_pollRXBuff = new u_char[m_slaveConfig.bPollResponseLength];
        memset(m_pollRXBuff,0x00,m_slaveConfig.bPollResponseLength);

        if(m_slaveConfig.bScanType == 1)
        {
            SetIAIIpFixedArea(m_pollTXBuff);//设置轮询报文的固定区域
            SegmentPollTXFrame(m_pollTXFrames, m_pollTXBuff, m_nSegmentNum);
        }
    }

    m_IAIFixedAreaStatus.IAI_RUN = OFF;
    m_IAIFixedAreaStatus.IAI_USE_SHAFT = OFF; /* 使用0-3 四个电缸轴 */

    for(auto &ShaftStatus:m_IAIShaftStatus)
    {
        ShaftStatus.IAI_Localization_Done = OFF;
        ShaftStatus.IAI_Origin_Return_Done = OFF;
        ShaftStatus.IAI_Moving = OFF;
        ShaftStatus.IAI_Alerting= OFF;
        ShaftStatus.IAI_Servo_On = OFF;
        ShaftStatus.IAI_Push_Press_Done = OFF;
        ShaftStatus.IAI_Collision = OFF;
        ShaftStatus.IAI_Overload_Alerting = OFF;

        ShaftStatus.IAI_MV_Command_Done = OFF;
        ShaftStatus.IAI_Get_Position_Data_Done = OFF;
        ShaftStatus.IAI_Teach_Mode = OFF;
        ShaftStatus.IAI_Ctrl_Ready = OFF;
        ShaftStatus.IAI_Emergency_stop = OFF;
    }

    ROS_INFO("[CDeviceNetMaster][AddSlave] end");
}

/*************************************************
Function: CDeviceNetMaster::StartScan
Description: 扫描检测IAI从站是否在线，是否可发起连接
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::StartScan()
{
    ROS_INFO("[CDeviceNetMaster][StartScan] begin...");

    /*ID:0x000005FE Standard  Data   DLC:0x06 data:0x 00 4B 03 01 01 00*/
    USB_CAN_OBJ UCMMReqFrame;
    uint16 UCMMReqConID = CalcConnectionID(UCMM_REQ);
    uint16 onlyGroupTwoConID = CalcConnectionID(ONLY_GROUP_2_REQ);
    while(ros::ok())
    {
        if(!m_bIsIAIUCMM)
        {
            for(int i=0;i<2;i++)
            {
                ROS_INFO("[CDeviceNetMaster][StartScan] send UCMM can frame:0x3F4B0234");
                setCanFrame(&UCMMReqFrame, UCMMReqConID, 4, 0x3F4B0234);
                usbCan_Transmit(&UCMMReqFrame, 1);

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                if (m_bIsIAIUCMM)
                    break;
            }
        }
        if (m_bIsIAIUCMM)
            break;
        if (!m_bIsIAIOnlyGroupTwo)
        {
            for (int i = 0; i < 2; i++)
            {
                ROS_INFO("[CDeviceNetMaster][StartScan] send only group_2 can frame:0x004B03010100");
                setCanFrame(&UCMMReqFrame, onlyGroupTwoConID, 6, 0x004B03010100);
                usbCan_Transmit(&UCMMReqFrame, 1);

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                if (m_bIsIAIOnlyGroupTwo)
                    break;
            }
        }
        if (m_bIsIAIOnlyGroupTwo)
            break;
    }
    ROS_INFO("[CDeviceNetMaster][StartScan] slave IAI is online, m_bIsIAIUCMM=%d, m_bIsIAIOnlyGroupTwo=%d.",m_bIsIAIUCMM, m_bIsIAIOnlyGroupTwo);
}

/*************************************************
Function: CDeviceNetMaster::UCMM
Description: UCMM 发起基于显示信息的连接请求
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::UCMM()
{
    ROS_INFO("[CDeviceNetMaster][UCMM] begin...");
    DeviceNetCon.SetInterfaceType(Display_Connection);//显示连接对象
    uint16 conID = 0;

    conID = CalcConnectionID(UCMM_REQ);
    ROS_INFO("[CDeviceNetMaster][UCMM] UCMM_REQ conID=%x",conID);//0x780
    DeviceNetCon.dIAIProduce.SetAttribute(nonentity, conID);

    conID = CalcConnectionID(UCMM_RESP);
    ROS_INFO("[CDeviceNetMaster][UCMM] UCMM_RESP conID=%x",conID);//0x77f
    DeviceNetCon.devIAIConsume.SetAttribute(nonentity, conID);

//组装UCMM连接请求报文
    ULL displayConFrame = 0;
    unsigned unServer = 0x4b;//请求服务编码
    unsigned unMsgType = 0x02;//信息体格式 16/16
    unsigned unGroup = 0x03;//信息组3
    unsigned unSourceID = 0x04;//源信息ID为4


    displayConFrame += (m_slaveConfig.dwMacID << 24);
    displayConFrame += (unServer << 16);
    displayConFrame += (unMsgType << 8);
    displayConFrame += (unGroup << 4);
    displayConFrame += unSourceID;//0x3F4B0234
    ROS_INFO("[CDeviceNetMaster][UCMM] displayConFrame=%llx",displayConFrame);

//UCMM发起连接
    BYTE dataLen = 4;
    std::vector<USB_CAN_OBJ> CanTempQueue;
    USB_CAN_OBJ canTemp;
//连接发送之前清除队列缓存
    std::unique_lock<std::mutex> lock(ReceiveMutex);
    shareCanData.clear();
    lock.unlock();

    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);

    if(CanTempQueue.size() != 1)
    {
        ROS_INFO("[CDeviceNetMaster][UCMM] UCMM connection error,received data size > 1.");
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }
    for(auto &canTempFrame:CanTempQueue)
    {
        canTemp = canTempFrame;
        if (!CheckUCMMConResp(&canTemp))
        {
            ROS_INFO("[CDeviceNetMaster][UCMM] UCMM connection error.");
            VCI_CloseDevice(VCI_USBCAN2,0);
            exit(-1);
        }
    }
    CanTempQueue.clear();

//从站正确响应后，获取到新的连接ID
    u_char sourceMsgID = canTemp.Data[3];//msgID
    printCanFrame(&canTemp, "[CDeviceNetMaster][UCMM]");
    ROS_INFO("[CDeviceNetMaster][UCMM] sourceMsgID=%02x",sourceMsgID);

    conID = CalcConnectionID(DISPLAY_PRODUCE, sourceMsgID);
    ROS_INFO("[CDeviceNetMaster][UCMM] DISPLAY_PRODUCE conID=%x",conID);//0x700
    DeviceNetCon.dIAIProduce.SetAttribute(exist, conID);

    conID = CalcConnectionID(DISPLAY_CONSUME, sourceMsgID);
    ROS_INFO("[CDeviceNetMaster][UCMM] DISPLAY_CONSUME conID=%x",conID);//0x73f
    DeviceNetCon.devIAIConsume.SetAttribute(exist, conID);

    ROS_INFO("[CDeviceNetMaster][UCMM] UCMM connection successful.");

/* 组装黑盒报文
    4  接收    15:42:16.747.0          0x00000700  数据帧  标准帧  0x08       3f 4b 03 00 01 00 02 00
    5  接收    15:42:16.762.0          0x0000073f  数据帧  标准帧  0x03       00 cb 02
    6  接收    15:42:16.762.0          0x00000700  数据帧  标准帧  0x07       3f 0e 01 00 01 00 01
    7  接收    15:42:16.762.0          0x0000073f  数据帧  标准帧  0x04       00 8e bb 02
    8  接收    15:42:16.762.0          0x00000700  数据帧  标准帧  0x07       3f 0e 01 00 01 00 02
    9  接收    15:42:16.762.0          0x0000073f  数据帧  标准帧  0x04       00 8e 00 00
    10  接收   15:42:16.762.0          0x00000700  数据帧  标准帧  0x07       3f 0e 01 00 01 00 03
    11  接收   15:42:16.762.0          0x0000073f  数据帧  标准帧  0x04       00 8e 6a 00
    12  接收   15:42:16.762.0          0x00000700  数据帧  标准帧  0x08       bf 00 10 05 00 02 00 09
    13  接收   15:42:16.762.0          0x0000073f  数据帧  标准帧  0x03       80 c0 00
    14  接收   15:42:16.762.0          0x00000700  数据帧  标准帧  0x04       bf 81 4b 00
    15  接收   15:42:16.762.0          0x0000073f  数据帧  标准帧  0x03       80 c1 00
    16  接收   15:42:16.778.0          0x0000073f  数据帧  标准帧  0x04       00 90 4c 00
    17  接收   15:42:16.778.0          0x00000700  数据帧  标准帧  0x07       3f 0e 05 00 02 00 07
    18  接收   15:42:16.778.0          0x0000073f  数据帧  标准帧  0x04       00 8e 20 00
    19  接收   15:42:16.778.0          0x00000700  数据帧  标准帧  0x07       3f 0e 05 00 02 00 08
    20  接收   15:42:16.778.0          0x0000073f  数据帧  标准帧  0x04       00 8e 20 00
    21  接收   15:42:16.778.0          0x00000780  数据帧  标准帧  0x04       3f 4c 0a 00
    22  接收   15:42:16.778.0          0x0000077f  数据帧  标准帧  0x02       00 cc

*/
//无脑发送黑盒报文
    dataLen = 8;
    displayConFrame = 0x3f4b030001000200;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();

    dataLen = 7;
    displayConFrame = 0x3f0e0100010001;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();
    displayConFrame = 0x3f0e0100010002 ;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();
    displayConFrame = 0x3f0e0100010003 ;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();

    dataLen = 8;
    displayConFrame = 0xbf00100500020009;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();

    dataLen = 4;
    displayConFrame = 0xbf814b00;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue, 2);
    CanTempQueue.clear();
    ROS_INFO("[CDeviceNetMaster][UCMM] DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue, 2)");//0x780

    dataLen = 7;
    displayConFrame = 0x3f0e0500020007;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();
    displayConFrame = 0x3f0e0500020008;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);
    CanTempQueue.clear();

//UCMM断开连接请求 3f 4c 0a 00

    conID = CalcConnectionID(UCMM_REQ);
    ROS_INFO("[CDeviceNetMaster][UCMM] UCMM_REQ conID=%x",conID);//0x780
    DeviceNetCon.dIAIProduce.SetAttribute(nonentity, conID);

    conID = CalcConnectionID(UCMM_RESP);
    ROS_INFO("[CDeviceNetMaster][UCMM] UCMM_RESP conID=%x",conID);//0x77f
    DeviceNetCon.devIAIConsume.SetAttribute(nonentity, conID);
    dataLen = 4;
    displayConFrame = 0x3f4c0a00;
    DeviceNetCon.dIAIProduce.SendData(dataLen, displayConFrame);

    CanTempQueue.clear();
    DeviceNetCon.devIAIConsume.ReadData(&CanTempQueue);

    if(CanTempQueue.size() != 1)
    {
        ROS_INFO("[CDeviceNetMaster][UCMM] UCMM disconnection error, received data size=%ld.",CanTempQueue.size());
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(-1);
    }
    for(auto &canTempFrame:CanTempQueue)
    {
        if (!CheckUCMMConResp(&canTempFrame))
        {
            ROS_INFO("[CDeviceNetMaster][UCMM] UCMM disconnection error.");
            VCI_CloseDevice(VCI_USBCAN2,0);
            exit(-1);
        }
    }

    ROS_INFO("[CDeviceNetMaster][UCMM] UCMM disconnection successful.");

    //从站正确响应后，设置对象的连接方式为IO连接，设置连接ID为轮询ID
    DeviceNetCon.SetInterfaceType(IO_Connection);//IO连接对象

    conID = CalcConnectionID(IO_PRODUCE);
    ROS_INFO("[CDeviceNetMaster][UCMM] IO_PRODUCE conID=%x",conID);//0x5fd
    DeviceNetCon.dIAIProduce.SetAttribute(exist, conID);

    conID = CalcConnectionID(IO_CONSUME);
    ROS_INFO("[CDeviceNetMaster][UCMM] IO_CONSUME conID=%x",conID);//0x3ff
    DeviceNetCon.devIAIConsume.SetAttribute(exist, conID);

    m_bPollFlag = true;//UCMM成功后置轮询标志位为true
    ROS_INFO("[CDeviceNetMaster][UCMM] end...");
}

/*************************************************
Function: CDeviceNetMaster::CalcConnectionID
Description: 计算拼接连接ID
Input: int IDType 连接ID的类型
       u_char msgID 连接ID对应的消息ID
Output: uint16 conID 连接ID
Others: void
**************************************************/
uint16 CDeviceNetMaster::CalcConnectionID(int IDType ,u_char msgID)
{
    uint16 conID = 0;
    switch(IDType)
    {
        case UCMM_REQ:
            /*拼接链接生产者的UCMM连接ID，组3消息*/
            /*1 1  组3信息ID  源MAC_ID(6位)  组 3 信息*/
            /*1 1  1 1 0     源MAC_ID  非连接的显式请求信息*/
            conID = 0x600;//3组can标识区的最高两位(10-11)信息固定为1
            conID += m_unMasterMacID;
            conID += (0x06 << 6);
            return conID;
        case UCMM_RESP:
            /*拼接链接消费者的UCMM连接ID，组3消息*/
            /*1 1  组3信息ID  源MAC_ID(6位)  组 3 信息*/
            /*1 1  1 0 1     源MAC_ID  非连接的显式响应信息*/
            conID = 0x600;//3组can标识区的最高两位(10-11)信息固定为1
            conID += m_slaveConfig.dwMacID;
            conID += (0x05 << 6);
            return conID;
        case DISPLAY_PRODUCE:
            /*拼接连接生产者的连接ID，组3消息*/
            conID = msgID << 6;
            conID += m_unMasterMacID;
            conID += 0x600;//3组can标识区的最高两位(10-11)信息固定为1
            return conID;
        case DISPLAY_CONSUME:
            /*拼接连消费者的连接ID，组3消息*/
            conID = msgID << 6;
            conID += m_slaveConfig.dwMacID;
            conID += 0x600;//3组can标识区的最高两位(10-11)信息固定为1
            return conID;
        case IO_PRODUCE:
            /*拼接链接生产者的连接ID，组2消息*/
            /*1 0    MAC_ID     组2信息ID   组2信息  400-5ff*/
            /*1 0    目的MAC_ID  1 0 1     主站 I/O 轮询命令/状态变化/循环信息*/
            conID = 0x5;
            conID += (m_slaveConfig.dwMacID << 3);//m_unMasterMacID;
            conID += 0x400;//2组的can标识区的最高1位(11位)信息固定为1
            return conID;
        case IO_CONSUME:
            /*拼接链接消费者的连接ID，组1消息*/
            /*0 组1信息ID  源MAC_ID  组1信息 000-3ff*/
            /*0 1 1 1 1   源MAC_ID  从站 I/O 轮询响应或状态变化循环应答信息*/
            conID = (0xf << 6);
            conID += m_slaveConfig.dwMacID;
            return conID;
        case ONLY_GROUP_2_REQ:
            /*拼接链接消费者的连接ID，组2消息*/
            /*1 0 MAC_ID     组2信息ID  组2信息 400-5ff*/
            /*1 0 目的MAC_ID  1 1 0    仅限组2无连接显式请求信息(预留)*/
            conID = 0x6;
            conID += (m_slaveConfig.dwMacID << 3);
            conID += 0x400;//2组的can标识区的最高1位(11位)信息固定为1
            return conID;
        case ONLY_GROUP_2_RESP:
            /*拼接链接消费者的连接ID，组2消息*/
            /*1 0 MAC_ID     组2信息ID  组2信息 400-5ff*/
            /*1 0 源 MAC_ID  0 1 1     从站的显式响应信息*/
            conID = 0x3;
            conID += (m_slaveConfig.dwMacID << 3);
            conID += 0x400;//2组的can标识区的最高1位(11位)信息固定为1
            return conID;
        default:
            return 0;
    }

}

/*************************************************
Function: CDeviceNetMaster::OnlyGroup2Con
Description: *仅限组2连接方式在直接建立I/O连接
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::OnlyGroup2Con()
{
    ROS_INFO("[CDeviceNetMaster][OnlyGroup2Con] begin...");//0x5fd
    uint16 conID;
    //从站正确响应后，设置对象的连接方式为IO连接，设置连接ID为轮询ID
    DeviceNetCon.SetInterfaceType(IO_Connection);//IO连接对象

    conID = CalcConnectionID(IO_PRODUCE);
    ROS_INFO("[CDeviceNetMaster][OnlyGroup2Con] IO_PRODUCE conID=%x",conID);//0x5fd
    DeviceNetCon.dIAIProduce.SetAttribute(exist, conID);

    conID = CalcConnectionID(IO_CONSUME);
    ROS_INFO("[CDeviceNetMaster][OnlyGroup2Con] IO_CONSUME conID=%x",conID);//0x3ff
    DeviceNetCon.devIAIConsume.SetAttribute(exist, conID);
    m_bPollFlag = true;//UCMM成功后置轮询标志位为true
    ROS_INFO("[CDeviceNetMaster][OnlyGroup2Con] end...");//0x5fd
}

/*************************************************
Function: CDeviceNetMaster::ReadSlaveConfigFile
Description: 读取从站初始化文件
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::ReadSlaveConfigFile()
{

}

/*************************************************
Function: CDeviceNetMaster::VerifyCanFrame
Description: 校验CAN帧数据
Input: USB_CAN_OBJ *canFrame 被检测的can帧数据
Output: int 1 有效数据
            0 无效数据
Others: void
**************************************************/
int CDeviceNetMaster::VerifyCanFrame(USB_CAN_OBJ *canFrame)
{
    if (canFrame->DataLen > 8) return 0; // error: data length

    if(canFrame->RemoteFlag || canFrame->ExternFlag)
    {
        ROS_INFO("[CDeviceNetMaster][VerifyCanFrame] Error,remote or extern frame");
        return 0;
    }

    return 1; // ext-frame ok
}

/*************************************************
Function: CDeviceNetMaster::VerifyCanFrame
Description: 校验CAN帧数据
Input: USB_CAN_OBJ *canFrame 被检测的can帧数据
Output: true  UCMM连接请求响应成功
        false UCMM连接请求响应失败
Others: void
**************************************************/
bool CDeviceNetMaster::CheckUCMMConResp(const USB_CAN_OBJ *canFrame)
{
    return (canFrame->Data[1] == 0xCB || canFrame->Data[1] == 0xCC);
}

/*************************************************
Function: CDeviceNetMaster::CheckIAIStatusThreadFunc
Description: 监测从站电缸控制器的状态
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::CheckIAIStatusThreadFunc()
{
    while(ros::ok())
    {
        if(m_tIAILastRespTime.tv_sec == 0 && m_tIAILastRespTime.tv_nsec == 0)
        {
//            ROS_INFO("[CDeviceNetMaster][CDeviceNetMaster] slave IAI has no pool-response.");
        }
        else if(m_bPollFlag)
        {
            timespec pollRespTime;
            long pollInterval;
            timespec_get(&pollRespTime, TIME_UTC);
            pollInterval=((pollRespTime.tv_sec - m_tIAILastRespTime.tv_sec) * 1000000000 \
                                 + (pollRespTime.tv_nsec - m_tIAILastRespTime.tv_nsec)) / 1000000;
            if(pollInterval > m_nIAITimeoutMilliSec)
            {
                ROS_INFO("[CDeviceNetMaster][CheckIAIStatusThreadFunc] pollInterval=%ld",pollInterval);
                ROS_INFO("[CDeviceNetMaster][CheckIAIStatusThreadFunc] slave IAI is offline");
                //掉线之后把轮询数据清空
                IAIShaftsMotion(IAI_SHAFT_INITIALISE);
                m_bIsIAIUCMM = false;
                m_bIsIAIOnlyGroupTwo = false;
                m_bIsIAIOnline = false;
                m_bPollFlag = false;
            }
        }

        if(!m_bIsIAIOnline)//掉线重连
        {
            StartScan();
            UCMM();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

/*************************************************
Function: CDeviceNetMaster::PollThreadFunction
Description: 轮询数据发送线程函数
             1.发送主站轮询报文
             2.处理轮询的接收报文
             3.当IAI的4个电机轴的控制器准备完成后开启伺服
Input: void
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::PollThreadFunction()
{
    ROS_INFO("[CDeviceNetMaster][PollThreadFunction] begin...");
    int slaveOn = OFF;

    while(ros::ok())
    {
        if(m_bPollFlag)
        {
            std::unique_lock<std::mutex> lock(m_PollTXMutex);
/*  //打印
            for(int i =0;i<m_nSegmentNum;i++)
            {
                PrintCanFrame(&m_pollTXFrames[i], "[CDeviceNetMaster][PollThreadFunction]");
            }
*/
            DeviceNetCon.dIAIProduce.SendData(m_pollTXFrames, m_nSegmentNum);
            lock.unlock();
            ManIAIPollResp();

            if(slaveOn == 0)//开启IAI4个电缸的伺服
            {
                int ctrlReady = 0;
//                for(int i=0;i<4;i++)
//                {
//                    ctrlReady += m_IAIShaftStatus[i].IAI_Ctrl_Ready;
//                }
                for(auto &shaftStatus:m_IAIShaftStatus)
                {
                    ctrlReady += shaftStatus.IAI_Ctrl_Ready;
                }
                if(ctrlReady == 4)
                {
                    IAIShaftsMotion(IAI_SHAFT_SLAVE_ON);
                    slaveOn = ON;
                    ROS_INFO("[CDeviceNetMaster][PollThreadFunction] IAI shaft slave is on");
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(m_nPollCycleTimeMilliSec));
        }
        else
        {
            slaveOn = OFF;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

    }

    ROS_INFO("[CDeviceNetMaster][PollThreadFunction] end...");
}

/*************************************************
Function: CDeviceNetMaster::SetPollDataPkg
Description: 配置电缸的运动报文的封装
             1.设置轮询发送BUFF中各个电缸轴的控制数据域
             2.将轮询发送BUFF按照分段协议封装到的CAN数据帧当中
Input: IAIShaftControl *ShaftControl 电缸控制命令结构体(包含几号电机,运动指令)
       int             ShaftCount    要控制的电缸的总个数,默认为1
Output: void
Others: void
**************************************************/
/*

*/
void CDeviceNetMaster::SetPollDataPkg(IAIShaftControl *ShaftControl, int ShaftCount)
{
/*定位器2 有效的控制帧数据
    "23		接收   15:42:16.793.0          0x000005fd  数据帧  标准帧  0x08       00 00 80 00 00 00 00 00 "
    "24		接收   15:42:16.793.0          0x000005fd  数据帧  标准帧  0x08       41 00 00 00 00 00 00 00 "
    "25		接收   15:42:16.793.0          0x000005fd  数据帧  标准帧  0x08       42 00 00 01 00 11 02 01 "
    "26		接收   15:42:16.793.0          0x000005fd  数据帧  标准帧  0x08       43 00 11 02 01 00 11 02 "
    "27		接收   15:42:16.793.0          0x000005fd  数据帧  标准帧  0x05       84 01 00 11 02 "
*/
    //设置轮询BUFF中各个电缸轴的控制数据域
    IAIShaftControl *Shafts = ShaftControl;
    for(int i=0;i<ShaftCount;i++)
    {
//        ROS_INFO("ShaftControl[%d] shaftNum=%d,command=0x%x",i,Shafts->shaftNum,Shafts->command);
        SetIAIShaftCommand(Shafts->shaftNum, Shafts->command, m_pollTXBuff);
        Shafts++;
    }

    std::unique_lock<std::mutex> lock(m_PollTXMutex);

    //将数据按照分段协议封装成can数据帧
    SegmentPollTXFrame(m_pollTXFrames, m_pollTXBuff, m_nSegmentNum);

    lock.unlock();
}

/*************************************************
Function: CDeviceNetMaster::SetIAIIpFixedArea
Description: 设置与IAI通讯报文的固定区域 共8字(16字节)
             定位器模式为:
              00 80 00 00
              00 00 00 00
              00 00 00 00
              00 00 00 00
Input: u_char *pollBuff 轮询报文buffer
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::SetIAIIpFixedArea(u_char *pollBuff)
{
    pollBuff[0] = 0x00;
    pollBuff[1] = 0x80;
    for(int i=2; i<16 ;i++)
    {
        pollBuff[i] = 0;
    }
}

/*************************************************
Function: CDeviceNetMaster::SetIAIShaftCommand
Description: IAI通讯报文以字(2个字节)为单位，但数据填入顺序为：低字节在前，高字节在后
             setIAIShaftCommand将电缸各个轴的控制报文转换后写入buff(32位u_char)中对应的位置
Input: uint   IAIShaftNUM  哪个电缸
       uint   ctrlCommand  控制指令
       u_char *pollBuff    轮询控制指令的buffer
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::SetIAIShaftCommand(uint IAIShaftNUM, uint ctrlCommand, u_char *pollBuff)
{
    int beginPos=0 , endPos=0;
    uint unCommand = 0;
    switch(IAIShaftNUM)
    {
        case 0:
            beginPos=16;
            endPos = 20;
            break;
        case 1:
            beginPos=20;
            endPos = 24;
            break;
        case 2:
            beginPos=24;
            endPos = 28;
            break;
        case 3:
            beginPos=28;
            endPos = 32;
            break;
        default:
            break;
    }
    //大小端数据问题
    unCommand += ((ctrlCommand >> 16) & 0xff);
    unCommand += ((ctrlCommand >> 16) & 0xff00);
    unCommand += ((ctrlCommand << 16) & 0xff0000);
    unCommand += ((ctrlCommand << 16) & 0xff000000);
    for(int i=beginPos; i<endPos ;i++)
    {
        pollBuff[i] = u_char(unCommand & 0xFF);
        unCommand >>= 8;
    }
}

/*************************************************
Function: CDeviceNetMaster::CalcSegmentNum
Description: 数据帧分段函数，计算分段个数
Input: BYTE commandLen 命令的总字节数
Output: void
Others: void
**************************************************/
int CDeviceNetMaster::CalcSegmentNum(BYTE commandLen)
{
    int segmentNum = 0;
    int bufLen = commandLen;
    if(bufLen < 8)
        return 1;
    segmentNum = bufLen / 7;
    if((bufLen % 7) > 0)
        segmentNum += 1;

    return segmentNum;
}

/*************************************************
Function: CDeviceNetMaster::SegmentPollTXFrame
Description: 按照分段协议，将32位buff数据拼接为对应的CAN帧
Input: USB_CAN_OBJ *canFrame     按照分段协议封装的can帧的首地址
       u_char      *pollBuff     数据源
       int         segmentNum    数据的总帧数
Output: void
Others: void
**************************************************/
/**/
void CDeviceNetMaster::SegmentPollTXFrame(USB_CAN_OBJ *canFrame, u_char *pollBuff, int segmentNum)
{
    int bufLen = m_slaveConfig.bPollCommandLength;
    if(segmentNum == 1)
    {
        canFrame[0].DataLen = (u_char)(bufLen+1);
        canFrame[0].Data[0] = 0x3f;
        for(int i=0; i< bufLen ; i++)
        {
            canFrame[0].Data[i+1] = pollBuff[i];
        }
    }
    else
    {
        int bufPos = 0, frameNum = 0;
        //第一帧数据
        canFrame[0].DataLen = 8;
        canFrame[0].Data[0] = 0x00;
        for(int i=1; i<8 && bufPos < bufLen ; i++, bufPos++)
        {
            canFrame[0].Data[i] = pollBuff[bufPos];
        }
        //中间帧数据
        frameNum++;

        for( ; frameNum < segmentNum; frameNum++)
        {
            if(frameNum == segmentNum -1)//最后帧数据
            {
                canFrame[frameNum].DataLen = u_char (bufLen % 7 + 1);
                canFrame[frameNum].Data[0] = u_char (0x80 + frameNum);
            }
            else
            {
                canFrame[frameNum].DataLen = 8;
                canFrame[frameNum].Data[0] = u_char (0x40 + frameNum);
            }

            for(int i=1; i < 8 && bufPos < bufLen; i++, bufPos++)
            {
                canFrame[frameNum].Data[i] = pollBuff[bufPos];
            }
        }
    }
}

/*************************************************
Function: CDeviceNetMaster::SegmentPollRXFrame
Description: 按照分段协议，计算各个分段can帧数据段的首字节,并填充到对应的buffer中.用于初始化轮询报文接收can帧数组
Input: int *segmentArray 轮询接收报文can帧数组首地址
       int  segmentNum   轮询报文分段总个数
Output: void
Others: void
**************************************************/
void CDeviceNetMaster::SegmentPollRXFrame(int *segmentArray, int segmentNum)
{
    if(segmentNum <= 1)
    {
        segmentArray[0] = 0x3f;
    }
    else
    {
        int frameNum = 0;
        //第一帧数据
        segmentArray[0] = 0x00;

        //中间帧数据
        frameNum++;

        for( ; frameNum < segmentNum; frameNum++)
        {
            if(frameNum == segmentNum -1)//最后帧数据
            {
                segmentArray[frameNum] = u_char (0x80 + frameNum);
            }
            else
            {
                segmentArray[frameNum] = u_char (0x40 + frameNum);
            }
        }
    }
}

/*************************************************
Function: CDeviceNetMaster::SegmentPollRXFrame
Description: 处理轮询的接收报文，读取对应位的值，并保存到状态变量当中
Input: void
Output: void
Others: void
**************************************************/
/**/
void CDeviceNetMaster::ManIAIPollResp()
{
    std::unique_lock<std::mutex> lock(m_PollRXMutex);

    /*0-15 固定域*/
    if( m_pollRXBuff[1] == 0x80)
    {
        m_IAIFixedAreaStatus.IAI_RUN = ON;
    }
    if( m_pollRXBuff[2] == 0x0F)
    {
        m_IAIFixedAreaStatus.IAI_USE_SHAFT = ON;
    }
//    printf("m_pollRXBuff:");
//    for(int i=16;i<32;i++)
//    {
//        printf(" %02x",m_pollRXBuff[i]);
//    }
//    printf("\n");

    /*16-19 轴0：16 17（17无效）位置信号，18 19 控制状态信号*/
    /*16-19 轴1：20 21（21无效）位置信号，22 23 控制状态信号*/
    /*16-19 轴2：24 25（25无效）位置信号，26 27 控制状态信号*/
    /*16-19 轴3：28 29（29无效）位置信号，30 31 控制状态信号*/
    int pos = 16;
    for(auto &ShaftStatus:m_IAIShaftStatus )
    {
        ShaftStatus.IAI_LocationNum =  m_pollRXBuff[pos];

        pos += 2;

        ShaftStatus.IAI_Localization_Done = (m_pollRXBuff[pos] & 0x01) ? 1:0;

        ShaftStatus.IAI_Origin_Return_Done = (m_pollRXBuff[pos] & 0x02) ? 1:0;

        ShaftStatus.IAI_Moving = (m_pollRXBuff[pos] & 0x04) ? 1:0;

        ShaftStatus.IAI_Alerting = (m_pollRXBuff[pos] & 0x08) ? 1:0;

        ShaftStatus.IAI_Servo_On = (m_pollRXBuff[pos] & 0x10) ? 1:0;

        ShaftStatus.IAI_Push_Press_Done = (m_pollRXBuff[pos] & 0x20) ? 1:0;

        ShaftStatus.IAI_Collision = (m_pollRXBuff[pos] & 0x40) ? 1:0;

        ShaftStatus.IAI_Overload_Alerting = (m_pollRXBuff[pos] & 0x80) ? 1:0;

        pos++;

        ShaftStatus.IAI_MV_Command_Done = (m_pollRXBuff[pos] & 0x01) ? 1:0;

        ShaftStatus.IAI_Get_Position_Data_Done = (m_pollRXBuff[pos] & 0x02) ? 1:0;

        ShaftStatus.IAI_Teach_Mode = (m_pollRXBuff[pos] & 0x04) ? 1:0;

        ShaftStatus.IAI_Ctrl_Ready = (m_pollRXBuff[pos] & 0x40) ? 1:0;

        ShaftStatus.IAI_Emergency_stop = (m_pollRXBuff[pos] & 0x80) ? 1:0;

//        ROS_INFO("m_IAIShaftStatus[%d].IAI_Ctrl_Ready=%d", i,ShaftStatus.IAI_Ctrl_Ready);
//        ROS_INFO("IAI_Localization_Done=%d,IAI_Origin_Return_Done=%d,IAI_Moving=%d,IAI_Alerting=%d,IAI_Servo_On=%d,"
//                 "IAI_Push_Press_Done=%d,IAI_Collision=%d,IAI_Overload_Alerting=%d",\
//                 ShaftStatus.IAI_Localization_Done,ShaftStatus.IAI_Origin_Return_Done,ShaftStatus.IAI_Moving,\
//                 ShaftStatus.IAI_Alerting,ShaftStatus.IAI_Servo_On,ShaftStatus.IAI_Push_Press_Done,\
//                 ShaftStatus.IAI_Collision,ShaftStatus.IAI_Overload_Alerting);
//        ROS_INFO("IAI_MV_Command_Done=%d,IAI_Get_Position_Data_Done=%d,IAI_Teach_Mode=%d,IAI_Ctrl_Ready=%d,"
//                 "IAI_Emergency_stop=%d",ShaftStatus.IAI_MV_Command_Done,ShaftStatus.IAI_Get_Position_Data_Done,\
//                 ShaftStatus.IAI_Teach_Mode,ShaftStatus.IAI_Ctrl_Ready,ShaftStatus.IAI_Emergency_stop);
        pos++;
    }
    lock.unlock();

}

/*************************************************
Function: CDeviceNetMaster::~CDeviceNetMaster
Description: CDeviceNetMaster类析构函数
Input: void
Output: void
Others: void
**************************************************/
CDeviceNetMaster::~CDeviceNetMaster()
{
    ROS_INFO("[CDeviceNetMaster][~CDeviceNetMaster] begin...");
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
#if USE_CAN2_RT
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    m_pCAN2Thread->join();
    delete m_pCAN2Thread;
#endif
    VCI_CloseDevice(VCI_USBCAN2, 0);
    ROS_INFO("[CDeviceNetMaster][~CDeviceNetMaster]VCI_CloseDevice done.");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    m_pRouteRecvThread->join();
    ROS_INFO("[CDeviceNetMaster][~CDeviceNetMaster]m_pRouteRecvThread end.");
    m_pRouteManThread->join();
    ROS_INFO("[CDeviceNetMaster][~CDeviceNetMaster]m_pRouteManThread end.");
//    m_pUCMMThread->join();
    m_pPollThread->join();
    ROS_INFO("[CDeviceNetMaster][~CDeviceNetMaster]m_pPollThread end.");
    m_pCheckIAIStatusThread->join();
    ROS_INFO("[CDeviceNetMaster][~CDeviceNetMaster]m_pCheckIAIStatusThread end.");
//    delete m_pUCMMThread;

    delete m_pRouteRecvThread;
    delete m_pRouteManThread;
    delete m_pPollThread;
    delete m_pCheckIAIStatusThread;
    delete m_nPollRXSegmentArray;
    delete[] m_pollTXFrames;
    delete[] m_pollRXFrames;
    delete[] m_pollTXBuff;
    delete[] m_pollRXBuff;
}

/*************************************************
Function: CDeviceNetMaster::IAIShaftsMotion
Description:按照电缸的控制协议,修改轮询报文,实现电缸具体的运动控制(电缸的运动的到的位置点或其他控制指令)
            对四个运动轴进行统一控制（目前控制需求比较单一，四个电缸运动保持一致即可，若想单独控制每个电缸的伸缩，可灵活调用SetPollDataPkg
Input: unsigned unPosNum  运动要到达的位置点 或者 其他控制指令(回原点,开启伺服,电机轴初始化等)
Output: void
Return: void
Others: void
**************************************************/
void CDeviceNetMaster::IAIShaftsMotion(unsigned unPosNum)
{
    if(!(m_IAIFixedAreaStatus.IAI_RUN && m_IAIFixedAreaStatus.IAI_USE_SHAFT) || !m_IAIShaftStatus->IAI_Ctrl_Ready)
    {
        ROS_INFO("[CDeviceNetMaster][IAIShaftsMotion] IAI is not ready, please control it latter.");
        return;
    }
    if(!m_IAIShaftStatus->IAI_Servo_On)
    {
        ROS_INFO("[CDeviceNetMaster][IAIShaftsMotion] IAI slave is not on.");
        return;
    }
    if(m_IAIShaftStatus->IAI_Moving)
    {
        ROS_INFO("[CDeviceNetMaster][IAIShaftsMotion] the shafts of IAI is moving, please control it latter.");
        return;
    }

    unsigned unMotionCommand = 0;
    int ShaftTotalNum = 4;
    IAIShaftControl ShaftControl[ShaftTotalNum];

    unMotionCommand = (unPosNum == IAI_ORIGIN_RETURN || unPosNum == IAI_SHAFT_SLAVE_ON || \
                       unPosNum == IAI_SHAFT_INITIALISE) ? unPosNum : IAIStartMoveToPos(unPosNum);

    for(int i = 0;i<ShaftTotalNum;i++)
    {
        ShaftControl[i].shaftNum = uint(i);
        ShaftControl[i].command = unMotionCommand;
    }
    SetPollDataPkg(ShaftControl, ShaftTotalNum);//运动开始
    if(unPosNum != IAI_ORIGIN_RETURN && unPosNum != IAI_SHAFT_SLAVE_ON && unPosNum != IAI_SHAFT_INITIALISE)//原点复位不需要发送“运动结束”指令
    {
        unMotionCommand = IAICompleteMoveToPos(unPosNum);
        std::this_thread::sleep_for(std::chrono::milliseconds(m_nPollCycleTimeMilliSec));
        for(int i = 0;i<ShaftTotalNum;i++)
        {
            ShaftControl[i].shaftNum = uint(i);
            ShaftControl[i].command = unMotionCommand;
        }
        SetPollDataPkg(ShaftControl, ShaftTotalNum);//运动结束
    }
}

/*************************************************
Function: CDeviceNetMaster::JoyCallBack
Description: 手柄控制
Input: const custom_msgs::JoyControlCmd::ConstPtr &JoyCommand 手柄msg
Output: void
Return: void
Others: void
**************************************************/
void CDeviceNetMaster::JoyCallBack(const custom_msgs::JoyControlCmd::ConstPtr &JoyCommand)
{
    //仅测试用
    ROS_INFO("[CDeviceNetMaster][JoyCallBack] get joy msg");
    unsigned unPosNum = 0;

    if (JoyCommand->cmd == "enable_fallen" || JoyCommand->cmd == "disable_fallen")
    {
        ROS_INFO("[CDeviceNetMaster][JoyCallBack] fallen move to position 1");
        unPosNum = 1;
        IAIShaftsMotion(unPosNum);
    }
    else if (JoyCommand->cmd == "disable_collision" || JoyCommand->cmd == "enable_collision")
    {
        ROS_INFO("[CDeviceNetMaster][JoyCallBack] collision move to position 2");
        unPosNum = 2;
        IAIShaftsMotion(unPosNum);
    }
    else if (JoyCommand->cmd == "query_status")
    {
        ROS_INFO("[CDeviceNetMaster][JoyCallBack] query_status origin return");
        unPosNum = IAI_ORIGIN_RETURN;
        IAIShaftsMotion(unPosNum);
    }
}
