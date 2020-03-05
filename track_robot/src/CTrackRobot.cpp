/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2019-2-25
Description: CTrackRobot
**************************************************/

#include "CTrackRobot.h"

double actual_tire_diameter;
double forward_diameter;
double back_diameter;


CTrackRobot::CTrackRobot():
    m_nDirection(FORWARD),
    m_nControlMode(TASK),
    m_nJoyCtrlTimeoutMSec(300),
    m_nAutoRunStatus(0),
    m_nBatteryPower(0),
    m_nBatteryStatus(0),
    m_nBatteryVoltage(0),
    m_nSdoCheckCount(0),
    m_nLightStatus(0),
    m_bMotorRunning(false),
    m_bIsMotorInit(false),
    m_bIsMotorPause(false),
    m_bIsSlowing(false),
    m_bIsInitializing(false),
    m_bCheckAlarmData(false),
    m_bGetAxesData(false),
    m_bForwardRelocation(true),
    m_bCheckSdo(true),
    m_bIsEStopButtonPressed(true),
    m_bIsUpdateOrign(false),
    m_dDistance(0),
    m_dOdom(0),
    m_dCurrentSpeed(0),
    m_dTargetSpeedTemp(0),
    m_dLimitDis(1.750),
    m_dPitchAngle(0.0),
    m_nMotorOneTemp(0),
    m_nMotorTwoTemp(0),
    m_sMotorOneStatus("normal"),
    m_sMotorTwoStatus("normal")
{
    ros::NodeHandle PublicNodeHandle;
    ros::NodeHandle PrivateNodeHandle("~");

    PrivateNodeHandle.param("can_speed", m_nCanSpeed, 500000);
    PrivateNodeHandle.param("print_can", m_nPrintCanRTX, 0);
    PrivateNodeHandle.param("imu_odom", m_nUseImuOdom, 1);
    PrivateNodeHandle.param("sport_mode", m_sMode, std::string("PP"));
    PrivateNodeHandle.param("motor_speed", m_dTargetSpeed, 0.0);
    PrivateNodeHandle.param("pv_a", m_dPvAcceleration, 0.2);
    PrivateNodeHandle.param("target_dis", m_dTargetDis, 10.0);
    PrivateNodeHandle.param("auto_run", m_nUseAutoRun, 1);
    PrivateNodeHandle.param("return_record", m_nReturnRecord, 0);
    PrivateNodeHandle.param("limit_dis", m_dLimitDis, 1.750);
    PrivateNodeHandle.param("scan_title", m_dScanTitle, 20.0);
    PrivateNodeHandle.param("timer_cycle", m_dTimerCycle, 10.0);
    PrivateNodeHandle.param("print_level", m_sPrintLevel, std::string("debug"));
    //added by btrmg for adjust mileage 2020.01.07
    PrivateNodeHandle.param("forward_diameter",forward_diameter,143.175);
    PrivateNodeHandle.param("back_diameter",back_diameter,143.175);
    //added end

    PublicNodeHandle.param("sub_cmd_topic", m_sSubCmdTopic, std::string("train_robot_control"));
    PublicNodeHandle.param("sub_joy_topic", m_sSubJoyTopic, std::string("train_joy"));
    PublicNodeHandle.param("sub_scan_topic", m_sSubScanTopic, std::string("scan"));
    PublicNodeHandle.param("sub_scan_topic", m_sSubImuTopic, std::string("imu0"));
    PublicNodeHandle.param("robot_odom", m_sSubOdomTopic, std::string("odom"));
    PublicNodeHandle.param("pub_pos_topic", m_sPubPositionTopic, std::string("train_robot_position"));
    PublicNodeHandle.param("pub_status_topic", m_sPubStatusTopic, std::string("train_robot_status"));
    PublicNodeHandle.param("track_odom", m_sPubOdomTopic, std::string("robot_odom"));
    PublicNodeHandle.param("pub_heart_beat_topic", m_sHeartBeatTopic, std::string("train_robot_heart_beat"));

    // Se the logging level manually to Debug, Info, Warn, Error
    ros::console::levels::Level printLevel = ros::console::levels::Info;
    if(m_sPrintLevel == "debug")
        printLevel = ros::console::levels::Debug;
    else if(m_sPrintLevel == "warn")
        printLevel = ros::console::levels::Warn;
    else if(m_sPrintLevel == "error")
        printLevel = ros::console::levels::Error;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, printLevel);

    ROS_INFO("usb can parameters:");

    ROS_INFO("[ros param] can_speed:%d", m_nCanSpeed);
    ROS_INFO("[ros param] print_can_flag:%d", m_nPrintCanRTX);
    ROS_INFO("[ros param] imu_odom:%d", m_nUseImuOdom);
    ROS_INFO("----------------------------------");
    ROS_INFO("motor control parameters:");
    ROS_INFO("[ros param] sport_mode:%s", m_sMode.c_str());
    ROS_INFO("[ros param] motor_speed:%f", m_dTargetSpeed);
    ROS_INFO("[ros param] pv_a:%f", m_dPvAcceleration);
    ROS_INFO("[ros param] target_dis:%f", m_dTargetDis);
    ROS_INFO("[ros param] auto_run:%d", m_nUseAutoRun);
    ROS_INFO("[ros param] return_record:%d", m_nReturnRecord);
    ROS_INFO("[ros param] limit_dis:%f", m_dLimitDis);
    ROS_INFO("[ros param] scan_title:%f", m_dScanTitle);
    ROS_INFO("[ros param] timer_cycle:%.1f", m_dTimerCycle);
    ROS_INFO("[ros param] print_level:%s", m_sPrintLevel.c_str());
    //added by btrmg for adjust mileage 2020.01.07
    ROS_INFO("[ros param] forward_diameter:%f", forward_diameter);
    ROS_INFO("[ros param] back_diameter:%f", back_diameter);
    //added end
    ROS_INFO("----------------------------------");
    ROS_INFO("ros topics:");
    ROS_INFO("[ros param] sub_cmd_topic:%s", m_sSubCmdTopic.c_str());
    ROS_INFO("[ros param] sub_joy_topic:%s", m_sSubJoyTopic.c_str());
    ROS_INFO("[ros param] sub_scan_topic:%s", m_sSubScanTopic.c_str());
    ROS_INFO("[ros param] sub_imu_topic:%s", m_sSubImuTopic.c_str());
    ROS_INFO("[ros param] pub_stat_topic:%s", m_sPubPositionTopic.c_str());
    ROS_INFO("[ros param] pub_status_topic:%s", m_sPubStatusTopic.c_str());
    ROS_INFO("[ros param] pub_heart_beat_topic:%s", m_sHeartBeatTopic.c_str());

    ROS_INFO("----------------------------------");

    if(m_sMode == "PP" && 1 != m_nUseImuOdom)
    {
        m_nMode = PP;
    }
    else
    {
        m_nMode = PV;
    }

    actual_tire_diameter = forward_diameter;

    m_dInitialAngle = 180.0-m_dScanTitle-(90.0-LASER_ANGLE_RANGE/2)-LASER_EXCISION_ANGLE;

    UsbCan.SetCanSpeed(m_nCanSpeed);

    if (!UsbCan.Init())
    {
        ROS_ERROR("init usbcan failed");
        exit(-1);
    }

    try
    {
        m_pCANReceiveThread = new std::thread(std::bind(&CTrackRobot::CANReceiveThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN receive thread failed, %s", exception.what());
        exit(-1);
    }

#if USE_CAN2_RT
    try
    {
        m_pCAN2ReceiveThread = new std::thread(std::bind(&CTrackRobot::CAN2ReceiveThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN2 receive thread failed, %s", exception.what());
        exit(-1);
    }
#endif

    try
    {
        m_pCANManageThread = new std::thread(std::bind(&CTrackRobot::CANManageThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN receive thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pCANSendThread = new std::thread(std::bind(&CTrackRobot::CANSendThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc CAN Send thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pControlThread = new std::thread(std::bind(&CTrackRobot::ControlThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc motor control thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pAutoRunThread = new std::thread(std::bind(&CTrackRobot::AutoRunThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc auto run thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pPositionFilterThread = new std::thread(std::bind(&CTrackRobot::ManageScanDataThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc filter Thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pCheckStatusThread = new std::thread(std::bind(&CTrackRobot::CheckStatusThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc filter Thread failed, %s", exception.what());
        exit(-1);
    }

    this_thread::sleep_for(std::chrono::milliseconds(500));

//    m_JoystickSubscriber = PublicNodeHandle.subscribe<custom_msgs::TrainJoyControlCmd>(m_sSubJoyTopic, 10, boost::bind(&CTrackRobot::JoystickCallBack, this, _1));
//    m_ScanSubscriber = PublicNodeHandle.subscribe<sensor_msgs::LaserScan>(m_sSubScanTopic, 10, boost::bind(&CTrackRobot::ScanCallBack, this, _1));
    m_ImuSubscriber = PublicNodeHandle.subscribe<sensor_msgs::Imu>(m_sSubImuTopic, 10, boost::bind(&CTrackRobot::ImuCallBack, this, _1));
    m_CommandSubscriber = PublicNodeHandle.subscribe<custom_msgs::TrainRobotControl>(m_sSubCmdTopic, 1, boost::bind(&CTrackRobot::CommandCallBack, this, _1));

    if(1 == m_nUseImuOdom)
    {
        m_OdomSubscriber = PublicNodeHandle.subscribe<nav_msgs::Odometry>(m_sSubOdomTopic, 1, boost::bind(&CTrackRobot::OdomCallBack, this, _1));
    }

    m_PositionPublisher = PublicNodeHandle.advertise<custom_msgs::TrainRobotPosition>(m_sPubPositionTopic, 10);
    m_StatusPublisher = PublicNodeHandle.advertise<custom_msgs::TrainRobotControlAck>(m_sPubStatusTopic, 10);
    m_HeartBeatPublisher = PublicNodeHandle.advertise<custom_msgs::TrainRobotHeartBeat>(m_sHeartBeatTopic, 10);
    m_OdomPublisher = PublicNodeHandle.advertise<nav_msgs::Odometry>(m_sPubOdomTopic, 10);

    m_BatteryService = PublicNodeHandle.advertiseService("battery_status", &CTrackRobot::BatteryServiceFunc, this);
    m_CurrentPosService = PublicNodeHandle.advertiseService("current_position", &CTrackRobot::CurrentPosServiceFunc, this);
    m_DisplacementTimer = PublicNodeHandle.createTimer(ros::Duration(m_dTimerCycle/1000.0), boost::bind(&CTrackRobot::DisplacementTimerFunc, this));

    this_thread::sleep_for(std::chrono::milliseconds(500));

    if(!m_bIsInitializing)
    {
        MotorServoInit();
    }

    try
    {
        m_pHeartBeatThread = new std::thread(std::bind(&CTrackRobot::HeartBeatThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc heart beat Thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pScanThread = new std::thread(std::bind(&CTrackRobot::ScanThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CTrackRobot::CTrackRobot] malloc scan Thread failed, %s", exception.what());
        exit(-1);
    }
}

/*************************************************
Function: CTrackRobot::CANReceiveThreadFunc
Description:　USBCAN设备数据接收线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CANReceiveThreadFunc()
{
    int i;
    ULL recLen;
    CanFrame rec[2500];
    while (ros::ok())
    {
        try
        {
//            if((recLen=UsbCan.Receive(rec, 2500, 100))>0)//这里二次封装后,会出现丢帧的问题,原因不明
            if((recLen=VCI_Receive(VCI_USBCAN2, 0, 0, rec, 2500, 0/*ms*/))>0)
            {
                if(recLen > 2500)
                {
                    UsbCan.Reset();
                    ROS_WARN("[CANReceiveThreadFunc] recLen = %lld",recLen);
                    continue;
                }
                std::unique_lock<std::mutex> lock(m_CanRXMutex);
                for(i=0; i<recLen; i++)
                {
                    if(rec[i].DataLen <=0 || rec[i].DataLen >8)
                        continue;

                    if(m_nPrintCanRTX)
                        UsbCan.PrintCanFrame(&rec[i], __FUNCTION__);
                    m_dCanRXDeque.push_back(rec[i]);
                }
                lock.unlock();
                m_CanRXCondition.notify_one();
            }
        }
        catch(...)
        {
            UsbCan.Reset();
            ROS_ERROR("[CANReceiveThreadFunc] catch can receive func error");
        }
    }
}

#if USE_CAN2_RT
/*************************************************
Function: CDeviceNetMaster::CAN2ReceiveThreadFunc
Description: 用于记录总线上的交互报文数据
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CAN2ReceiveThreadFunc()
{
    ROS_DEBUG("[CAN2ReceiveThreadFunc] running");
    int i;
    ULL recLen;
    CanFrame rec[3000];
    string sDataFileName = m_sDataFilePath;

    sDataFileName += "servo/";
    sDataFileName += "ServoInit";
    CFileRW ServoInitFileRw;
    time_t t = time(nullptr);
    char cTmp[64];
    strftime( cTmp, sizeof(cTmp), "%Y%m%d-%H%M%S",localtime(&t));
    sDataFileName += cTmp;
    sDataFileName += ".txt";

    ServoInitFileRw.OpenFile(sDataFileName, std::string("a+"));

    ROS_DEBUG("[CreateNewRecordFile] open file:%s",sDataFileName.c_str());
    while (ros::ok())
    {
        if(!m_bIsMotorInit)
        {
            if((recLen=VCI_Receive(VCI_USBCAN2, 0, 1, rec, 3000, 100/*ms*/))>0)
            {
                for(i=0; i<recLen; i++)
                {
                    if(rec[i].DataLen <=0 || rec[i].DataLen >8)
                        continue;

                    string sPrintStr;
                    char buf[32];

                    timeval tv;
                    char cTimeTmp[64];

                    gettimeofday(&tv, nullptr);
                    strftime(cTimeTmp, sizeof(cTimeTmp)-1, "TimeStamp:%Y/%m/%d-%H:%M:%S", localtime(&tv.tv_sec));
                    sprintf(buf, "%s.%03d ", cTimeTmp, (int)(tv.tv_usec / 1000));
                    sPrintStr += buf;

                    sprintf(buf,"ID:0x%08X ", rec[i].ID);//ID
                    sPrintStr += buf;

                    if(rec[i].ExternFlag==0) sprintf(buf, "Standard ");//帧格式：标准帧
                    if(rec[i].ExternFlag==1) sprintf(buf, "Extend   ");//帧格式：扩展帧
                    sPrintStr += buf;

                    if(rec[i].RemoteFlag==0) sprintf(buf, "Data   ");//帧类型：数据帧
                    if(rec[i].RemoteFlag==1) sprintf(buf, "Remote ");//帧类型：远程帧
                    sPrintStr += buf;

                    sprintf(buf, "Len:%d", rec[i].DataLen);//帧长度
                    sPrintStr += buf;

                    sPrintStr += " data:0x";	//数据
                    for(int j = 0; j < rec[i].DataLen; j++)
                    {
                        sprintf(buf, " %02X", rec[i].Data[j]);
                        sPrintStr += buf;
                    }

                    sPrintStr += "\n";
                    ServoInitFileRw.Output(sPrintStr);
                }
            }
        }
        else
        {
            ServoInitFileRw.CloseFile();
            break;
        }

    }
}
#endif

/*************************************************
Function: CTrackRobot::CANManageThreadFunc
Description:　USBCAN设备数据处理线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CANManageThreadFunc()
{
    ROS_INFO("[CANManageThreadFunc] start");

    CanFrame canTemp;
    std::deque<CanFrame> vCanFrameDeque;
    unsigned long ulSize;

    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_CanRXMutex);
        while(m_dCanRXDeque.empty())
        {
            m_CanRXCondition.wait(lock);
        }

        vCanFrameDeque.assign(m_dCanRXDeque.begin(), m_dCanRXDeque.end());
        m_dCanRXDeque.clear();
        lock.unlock();

        ulSize = vCanFrameDeque.size();
        for (int i = 0; i < ulSize; i++)
        {
            canTemp = vCanFrameDeque[i];
            //PDO Motor INFO
            if(canTemp.u32Id == m_MotorOne.m_unPdoRxCobId)
            {
                timespec_get(&m_tTimeOfLastRecvCan, TIME_UTC);
                m_MotorOne.UpdateMotorStatus(canTemp);

                m_CurrentMotorPos.nMotorOnePos = m_MotorOne.m_MotorStatus.nCurrentPos;
                m_dCurrentSpeed1 = m_MotorOne.m_MotorStatus.dActual_speed;
                m_bMotorRunning = (abs(m_dCurrentSpeed) > 0.001);
            }
            else if(canTemp.u32Id == m_MotorTwo.m_unPdoRxCobId)
            {
                timespec_get(&m_tTimeOfLastRecvCan, TIME_UTC);
                m_MotorTwo.UpdateMotorStatus(canTemp);

                m_CurrentMotorPos.nMotorTwoPos = m_MotorTwo.m_MotorStatus.nCurrentPos;
                m_dCurrentSpeed = (m_MotorTwo.m_MotorStatus.dActual_speed + m_dCurrentSpeed1)/2;
                m_bMotorRunning = (abs(m_dCurrentSpeed) > 0.001);

                if(m_nMode == PP)
                {
                    m_bMotionEnd = (m_MotorTwo.m_MotorStatus.nServoStatus & 0x40) > 0;

                    CanFrame SpeedCanData;
                    SpeedCanData.ID = m_MotorOne.m_unPdoTxCobId;
                    SpeedCanData.RemoteFlag = 0;
                    SpeedCanData.ExternFlag = 0;
                    SpeedCanData.SendType = 0;
                    SpeedCanData.DataLen = 2;
                    SpeedCanData.Data[0] = canTemp.Data[0];
                    SpeedCanData.Data[1] = canTemp.Data[1];

                    SendCanFrame(&SpeedCanData, 1);
                }

                if(1 != m_nUseImuOdom)
                {
                    if(m_bMotorRunning)
                    {
                        CalcRobotMileage();
                    }
                }
            }

            //Motor Sdo Check
            else if(canTemp.u32Id == m_MotorOne.m_unSdoRxCobId || canTemp.u32Id == m_MotorTwo.m_unSdoRxCobId)
            {
                if(m_bCheckSdo)
                {
                    std::unique_lock<std::mutex> Lock(m_SdoCheckMutex);
                    if(UsbCan.CanFrameCmp(&m_SdoCheckBuf[m_nSdoCheckCount],&canTemp) != 0)
                    {
                        UsbCan.PrintCanFrame(&m_SdoCheckBuf[m_nSdoCheckCount],"SdoCheckBuf");
                        UsbCan.PrintCanFrame(&canTemp,"recvCanData");
                        m_bCheckSdo = false;
                    }
                    m_nSdoCheckCount++;
                }
                else if(canTemp.u32Id == m_MotorOne.m_unSdoRxCobId)
                {
                    m_MotorOne.ManageSdoInfo(canTemp);
                    m_sMotorOneStatus = m_MotorOne.m_MotorStatus.sErr;
                    if(m_sMotorOneStatus != "normal" && m_bIsMotorInit)
                    {
                        ROS_ERROR("[CANManageThreadFunc] Motor1 servo error");
                        m_dTargetSpeed = 0;
                        m_bIsMotorInit = false;

                        if(GpioControl(G_LIGHT_OFF) == -1)
                            ROS_ERROR("[MotorServoInit] turn off green light failed");
                        if(GpioControl(R_LIGHT_ON) == -1)
                            ROS_ERROR("[MotorServoInit] turn on red light failed");
                    }
                }
                else if(canTemp.u32Id == m_MotorTwo.m_unSdoRxCobId)
                {
                    m_MotorTwo.ManageSdoInfo(canTemp);
                    m_sMotorTwoStatus = m_MotorTwo.m_MotorStatus.sErr;

                    if(m_sMotorTwoStatus != "normal" && m_bIsMotorInit)
                    {
                        ROS_ERROR("[CANManageThreadFunc] Motor2 servo error");
                        m_dTargetSpeed = 0;
                        m_bIsMotorInit = false;

                        if(GpioControl(G_LIGHT_OFF) == -1)
                            ROS_ERROR("[MotorServoInit] turn off green light failed");
                        if(GpioControl(R_LIGHT_ON) == -1)
                            ROS_ERROR("[MotorServoInit] turn on red light failed");
                    }
                }
            }

//字节1 电池电压(单位0.1V,400->40V);字节2 电池电压(单位0.1V,400->40V);字节3 电池电流(单位0.1A,200->20A);字节4 电池电流(单位0.1A,200->20A)
//字节5 电量信息(SOC,0-100);字节6	电池温度(65表示25℃,负偏40);字节7 充电状态：1正在充电，0没有充电;字节8	电池组串数
            else if(canTemp.u32Id == 0xBB)
            {
                m_nBatteryVoltage = canTemp.szData[0];
                m_nBatteryVoltage <<= 8;
                m_nBatteryVoltage += canTemp.szData[1];

                m_nBatteryCurrent = canTemp.szData[2];
                m_nBatteryCurrent <<= 8;
                m_nBatteryCurrent += canTemp.szData[3];

                m_nBatteryPower = canTemp.szData[4];
                m_nBatteryTemp = canTemp.szData[5] - 40;
                m_nBatteryCharge = canTemp.szData[6];
            }
//#电池状态：bit0：欠压 bit1：过压 bit2：过流 bit3：低温 bit4：高温 bit5：充电中 bit6：放电中 bit7：充电fet损坏
            else if(canTemp.u32Id == 0xBE)
            {
                m_nBatteryStatus = 0x00;
                m_nBatteryStatus |= (canTemp.szData[1] & 0xAA)? 0x01:0x00;//欠压
                m_nBatteryStatus |= (canTemp.szData[1] & 0x55)? 0x02:0x00;//过压
                m_nBatteryStatus |= (canTemp.szData[3] & 0x34)? 0x04:0x00;//过流

                m_nBatteryStatus |= (canTemp.szData[4] & 0xAA)? 0x08:0x00;
                m_nBatteryStatus |= (canTemp.szData[5] & 0xAA)? 0x08:0x00;//低温

                m_nBatteryStatus |= (canTemp.szData[4] & 0x55)? 0x10:0x00;
                m_nBatteryStatus |= (canTemp.szData[5] & 0x55)? 0x10:0x00;//高温

                m_nBatteryStatus |= (canTemp.szData[3] & 0x01)? 0x20:0x00;//充电状态
//                m_nBatteryStatus |= (canTemp.szData[3] & 0x02)? 0x40:0x00;//放电状态
                m_nBatteryStatus = 0;//放电状态
                m_nBatteryStatus |= (canTemp.szData[7] & 0x02)? 0x80:0x00;//充电fet损坏
            }
        }
        vCanFrameDeque.clear();
    }
}

/*************************************************
Function: CTrackRobot::CANSendThreadFunc
Description:　USBCAN设备数据发送线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CANSendThreadFunc()
{
    ROS_INFO("[CANSendThreadFunc] start");
    CanFrame canFrame;
    std::deque<CanFrame> CanSendDeque;
    unsigned long ulSize = 0;

    while (ros::ok())
    {
        try
        {
            std::unique_lock<std::mutex> LockSendPack(m_CanTXMutex);
            if (m_dCanTXDeque.empty())
            {
                m_CanTXCondition.wait(LockSendPack);
            }

            CanSendDeque.assign(m_dCanTXDeque.begin(), m_dCanTXDeque.end());
            m_dCanTXDeque.clear();
            LockSendPack.unlock();

            ulSize = CanSendDeque.size();
            if(ulSize > 100)
            {
                ROS_WARN("[CANSendThreadFunc] send buf size is: %ld.",ulSize);
            }
            for (int i = 0; i < ulSize; i++)
            {
                canFrame = CanSendDeque[i];

                if(m_nPrintCanRTX)
                    UsbCan.PrintCanFrame(&canFrame, __FUNCTION__);
                UsbCan.SendCan(&canFrame, 1);

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        catch(...)
        {
            UsbCan.Reset();
            ROS_ERROR("[CANSendThreadFunc] catch can send func error");
        }

    }
}

/*************************************************
Function: CTrackRobot::ControlThreadFunc
Description:　电机运动控制线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ControlThreadFunc()
{
    char c;
    while(ros::ok())
    {
        cin >> c;
        if(c == '\n')
            continue;

        if(c == 'q')
        {
            ROS_DEBUG("[ControlThreadFunc] process quit.");
            m_bMotorRunning = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            UsbCan.Close();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            exit(-1);
        }
        else if(c == 'i')
        {
            ROS_DEBUG("[ControlThreadFunc] servo init");
            MotorServoInit();
        }
        else if(c == 'c')
        {
            ROS_DEBUG("[ControlThreadFunc] clear servo error");
            ClearWarn();
        }
        else if(m_bIsMotorInit)
        {
            if(c == 's')
            {
                PublishStatus(STOP);
                ROS_DEBUG("[CommandCallBack] stop cmd.");
                if(!m_bMotorRunning)
                {
                    ROS_DEBUG("[CommandCallBack] robot has stopped, return.");
                }
                else if(m_bIsSlowing)
                {
                    m_bIsSlowing = false;
                    ROS_DEBUG("[CommandCallBack] motor is slowing, return.");
                }
                else
                {
                    StopMotion();
                }
            }
            else if(c == 'g')
            {
                ResumeMotion();
            }
            else if(c == 'a')
            {
                ROS_DEBUG("[ControlThreadFunc] send speed-up can frame.");
                MotorSpeedUp();
            }
            else if(c == 'd')
            {
                ROS_DEBUG("[ControlThreadFunc] send speed-down can frame.");
                MotorSpeedDown();
            }
            else if(c == 'S')
            {
                ROS_DEBUG("[ControlThreadFunc] send motor power off can frame.");
                MotorPowerOff();
            }
            else if(c == 'p')
            {
                ROS_DEBUG("[ControlThreadFunc] m_dDistance=%f,m_dTargetSpeed,=%f,m_dCurrentSpeed=%f.",m_dDistance,m_dTargetSpeed, m_dCurrentSpeed);
                ROS_DEBUG("[CommandCallBack] m_dInitialAngle=%f,m_dPitchAngle=%f",m_dInitialAngle, m_dPitchAngle);
            }
            else if(c == '1')
            {
                ROS_DEBUG("[ControlThreadFunc] m_dDistance=%f,m_dTargetSpeed,=%f,m_dCurrentSpeed=%f.",m_dDistance,m_dTargetSpeed, m_dCurrentSpeed);
                ROS_DEBUG("[CommandCallBack] m_dInitialAngle=%f,m_dPitchAngle=%f",m_dInitialAngle, m_dPitchAngle);
            }
        }
        else
        {
            ROS_DEBUG("[ControlThreadFunc] Motor is not init, c=%c",c);
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*************************************************
Function: CTrackRobot::AutoRunThreadFunc
Description:　自动运动线程功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::AutoRunThreadFunc()
{
    ROS_INFO("[AutoRunThreadFunc] AutoRunThreadFunc Running.");
    while(ros::ok())
    {
        if(m_nControlMode == TASK)
        {
            if(m_bMotorRunning)
            {
                if(m_dDistance >= m_dTargetDis - AUTO_RUN_LOCATION_TRIM && m_nAutoRunStatus == INITIAL)
                {
                    m_nAutoRunStatus = LOCATION_DONE;

                    if(m_nMode == PV)
                    {
                        ROS_INFO("[AutoRunThreadFunc] run Quick Stop");
                        QuickStop();
                        this_thread::sleep_for(std::chrono::milliseconds(200));
                    }

                    if(!m_nUseAutoRun)
                    {
                        this_thread::sleep_for(std::chrono::milliseconds(5000));
                        PublishStatus(TASK_DONE);
                    }
                    else if(m_nUseAutoRun)
                    {
                        this_thread::sleep_for(std::chrono::milliseconds(3000));
                        ROS_INFO("[AutoRunThreadFunc] ReturnToOrigin");
                        ReturnToOrigin(true);
                    }
                    ROS_DEBUG("[AutoRunThreadFunc] PV forward or backward m_dDistance=%lf",m_dDistance);
                }
                else if(m_dDistance <= AUTO_RUN_LOCATION_TRIM && m_nDirection >= FORWARD_RETURN && m_nAutoRunStatus != INITIAL)
                {
                    m_nAutoRunStatus = INITIAL;

                    if(m_nMode == PV)
                    {
                        ROS_INFO("[AutoRunThreadFunc] return Quick Stop");
                        QuickStop();
                    }
                    this_thread::sleep_for(std::chrono::milliseconds(2000));
                    ROS_DEBUG("[AutoRunThreadFunc] PV return m_dDistance=%lf",m_dDistance);

                    PublishStatus(TASK_DONE);
                }
            }

        }
        else if(m_nControlMode == ALARM_LOCATION)
        {
            if(m_nMode == PV && m_bMotorRunning)
            {
                if(m_dDistance >= m_dAlarmTargetDis - AUTO_RUN_LOCATION_TRIM + LIMIT_LOCATION_RANGE_PARAM && m_nAutoRunStatus == INITIAL)
                {
                    ROS_INFO("[AutoRunThreadFunc] alarm location Quick Stop");
                    QuickStop();
                    ROS_DEBUG("[AutoRunThreadFunc] alarm location m_dDistance=%0.4f",m_dDistance);
                    PublishStatus(RELOCATION_DONE);
                    m_nAutoRunStatus = LOCATION_DONE;
                    m_bCheckAlarmData = false;
                    this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                else if(m_dDistance <= AUTO_RUN_LOCATION_TRIM && m_nDirection >= FORWARD_RETURN)
                {
                    QuickStop();
                    m_nAutoRunStatus = INITIAL;
                    PublishStatus(RETURN_DONE);
                }
            }
        }
        else if(m_nMode == PV && m_bMotorRunning && m_nControlMode == JOY &&
                m_dDistance <= AUTO_RUN_LOCATION_TRIM && m_nDirection >= FORWARD_RETURN)
        {
            QuickStop();
            m_nAutoRunStatus = INITIAL;
            PublishStatus(RETURN_DONE);
        }

        this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

/*************************************************
Function: CTrackRobot::DisplacementTimerFunc
Description:　里程计算及消息发布定时器功能函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::DisplacementTimerFunc()
{
    if(1 == m_nUseImuOdom)
    {
        nav_msgs::Odometry OdomMsg;

        OdomMsg.header.stamp = ros::Time::now();
        OdomMsg.header.frame_id = "odom";

        if(m_bIsEStopButtonPressed)
        {
            m_dCurrentSpeed = 0;
        }

        OdomMsg.twist.twist.linear.x = m_dCurrentSpeed;

        OdomMsg.twist.covariance[0] = 1e-8;
        OdomMsg.twist.covariance[7] = 1e-8;
        OdomMsg.twist.covariance[14] = 1e-8;
        OdomMsg.twist.covariance[21] = 1e-8;
        OdomMsg.twist.covariance[28] = 1e-8;
        OdomMsg.twist.covariance[35] = 1e-8;

        m_OdomPublisher.publish(OdomMsg);
    }

    if(!m_bIsMotorInit || m_bIsEStopButtonPressed)
        return;

    //非位置信息的速度处理:匀加速 or 匀减速
    if(m_nMode == PV)
        AccelerationOrDeceleration();

    SendGetMotorInfoCmd();
}

/*************************************************
Function: CTrackRobot::AccelerationOrDeceleration
Description: 匀加速 and 匀减速
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::AccelerationOrDeceleration()
{
    std::unique_lock<std::mutex> Lock(m_CurrentSpeedMutex);

    double dEndSpeed = 0.05;
    if(m_nControlMode == TASK || m_nControlMode == ALARM_LOCATION || (m_nControlMode == JOY && m_nDirection >= FORWARD_RETURN))
    {
        if(!m_bIsMotorPause && abs(m_dTargetSpeed) > dEndSpeed &&\
            ((m_nDirection >= FORWARD_RETURN && m_dDistance <= m_dEndDis) ||\
             (m_nDirection <= BACKWARD &&(m_dDistance >= (m_dTargetDis - m_dEndDis) || \
                                          (m_nControlMode == ALARM_LOCATION && m_dDistance >= (m_dAlarmTargetDis - m_dEndDis))))))
        {
            if(m_nDirection == FORWARD || m_nDirection == BACKWARD_RETURN)
            {
                m_dTargetSpeed = dEndSpeed;
                ROS_DEBUG("[AccelerationOrDeceleration] deceleration set speed:%f",m_dTargetSpeed);
            }
            else if(m_nDirection == BACKWARD || m_nDirection == FORWARD_RETURN)
            {
                m_dTargetSpeed = -dEndSpeed;
                ROS_DEBUG("[AccelerationOrDeceleration] deceleration set speed:%f",m_dTargetSpeed);
            }
        }
        else
            m_bIsSlowing = false;
    }

    SetMotorSpeed(m_dTargetSpeed);

//    if(m_nControlMode == ALARM_LOCATION && m_bMotorRunning && m_nDirection <= BACKWARD)
//    {
//        if(m_dEndDis > LIMIT_LOCATION_RANGE_PARAM)
//            m_bCheckAlarmData = (m_dDistance >= m_dAlarmTargetDis - LIMIT_LOCATION_RANGE_PARAM);
//        else
//            m_bCheckAlarmData = (m_dDistance >= m_dAlarmTargetDis-m_dEndDis) && (abs(m_dCurrentSpeed)<=0.2);
//    }

    Lock.unlock();
}

/*************************************************
Function: CTrackRobot::CalcRobotMileage
Description: 换算机器人运动里程
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CalcRobotMileage()
{
    int nMotor_One_Displacement=0, nMotor_Two_Displacement=0;

    //32位数值溢出
    if(m_nDirection == FORWARD || m_nDirection == FORWARD_RETURN)
    {
        if(m_nMode == PV)
        {
            if(m_CurrentMotorPos.nMotorOnePos < m_MotorPos.nMotorOnePos)
            {
                nMotor_One_Displacement = 0x7FFFFFFF - m_MotorPos.nMotorOnePos;
                nMotor_One_Displacement += m_CurrentMotorPos.nMotorOnePos - 0x80000000 + 1;
            }
            else
            {
                nMotor_One_Displacement = m_CurrentMotorPos.nMotorOnePos-m_MotorPos.nMotorOnePos;
            }
        }

        if(m_CurrentMotorPos.nMotorTwoPos < m_MotorPos.nMotorTwoPos)
        {
            nMotor_Two_Displacement = 0x7FFFFFFF - m_MotorPos.nMotorTwoPos;
            nMotor_Two_Displacement += m_CurrentMotorPos.nMotorTwoPos - 0x80000000 + 1;
        }
        else
        {
            nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_MotorPos.nMotorTwoPos;
        }
    }
    else if(m_nDirection == BACKWARD || m_nDirection == BACKWARD_RETURN)
    {
        if(m_nMode == PV)
        {
            if(m_CurrentMotorPos.nMotorOnePos > m_MotorPos.nMotorOnePos)
            {
                nMotor_One_Displacement = m_MotorPos.nMotorOnePos - 0x80000000;
                nMotor_One_Displacement += 0x7FFFFFFF - m_CurrentMotorPos.nMotorOnePos + 1;
            }
            else
            {
                nMotor_One_Displacement = m_MotorPos.nMotorOnePos-m_CurrentMotorPos.nMotorOnePos;
            }
        }

        if(m_CurrentMotorPos.nMotorTwoPos > m_MotorPos.nMotorTwoPos)
        {
            nMotor_Two_Displacement = m_MotorPos.nMotorTwoPos - 0x80000000;
            nMotor_Two_Displacement += 0x7FFFFFFF - m_CurrentMotorPos.nMotorTwoPos + 1;
        }
        else
        {
            nMotor_Two_Displacement = m_MotorPos.nMotorTwoPos-m_CurrentMotorPos.nMotorTwoPos;
        }
    }

    if(m_nMode == PV)
    {
        double dDistance = (nMotor_One_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
        m_dDistance = ((nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000 + dDistance) / 2;
    }
    else if(m_nMode == PP)
    {
        m_dDistance = (nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
    }

    //	if(m_nDirection == BACKWARD)
//		m_dDistance *=1.000;
//	else
//		m_dDistance *=1.001;
//    ROS_DEBUG("[DisplacementTimerFunc] m_dDistance=%f",m_dDistance);
}

/*************************************************
Function: CTrackRobot::MotorServoInit
Description:　电机伺服初始化函数
Input: void
Output: void
Others: void
**************************************************/
int CTrackRobot::MotorServoInit()
{
    ROS_DEBUG("[MotorServoInit] start.");
    m_bIsMotorInit = false;

    if(GpioControl(R_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off red light failed");
        return -1;
    }
    if(GpioControl(G_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off green light failed");
        return -1;
    }
    if(GpioControl(B_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off blue light failed");
        return -1;
    }
    if(GpioControl(R_LIGHT_ON) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn on red light failed");
        return -1;
    }

    if(m_bIsEStopButtonPressed)
    {
        ROS_WARN("[MotorServoInit] motor init failed, the E-Stop button is pressed");
        return -1;
    }

    m_bIsInitializing = true;
    if(RestartMotorServo() == -1)
    {
        ROS_ERROR("[MotorServoInit] start motor servo failed");
        m_bIsInitializing = false;
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(1000));

    m_nControlMode = TASK;
    m_MotorPos.nMotorOnePos = 0;
    m_MotorPos.nMotorTwoPos = 0;

    m_tTimeOfLastJoyCmd.tv_sec = 0;
    m_tTimeOfLastJoyCmd.tv_nsec = 0;

    m_tTimeOfLastRecvCan.tv_sec = 0;
    m_tTimeOfLastRecvCan.tv_nsec = 0;

    m_MotorOne.SetMotorId(1);
    m_MotorTwo.SetMotorId(2);

    m_MotorOne.SetMotorControlMode(PV);
    m_MotorTwo.SetMotorControlMode(m_nMode);

    int nCanDataLen = 40;
    int nCount = 0;
    bool bIsCheck = true;

    CanFrame initCanData[nCanDataLen];
    CanFrame initCheckCanData[nCanDataLen];

    memset(initCanData, 0, sizeof(CanFrame)*nCanDataLen);
    memset(initCheckCanData, 0, sizeof(CanFrame)*nCanDataLen);

    SendCanFrame(0, 1, 0x01);//0x01 启动节点

    m_MotorOne.InitMotorCmdPack(initCanData);
    nCount = m_MotorOne.InitMotorCheckDataPack(initCheckCanData);

    if(SendCanFrame(initCanData, nCount, initCheckCanData, bIsCheck) == -1)
    {
        ROS_ERROR("[MotorServoInit] motor one init error");
        m_bIsInitializing = false;
        return -1;
    }

    memset(initCanData, 0, sizeof(CanFrame)*nCanDataLen);
    memset(initCheckCanData, 0, sizeof(CanFrame)*nCanDataLen);

    m_MotorTwo.InitMotorCmdPack(initCanData);
    nCount = m_MotorTwo.InitMotorCheckDataPack(initCheckCanData);

    if(SendCanFrame(initCanData, nCount, initCheckCanData, bIsCheck) == -1)
    {
        ROS_ERROR("[MotorServoInit] motor two init error");
        m_bIsInitializing = false;
        return -1;
    }

    if(SetRobotAcceleration(m_dPvAcceleration) == -1)
    {
        ROS_ERROR("[MotorServoInit] init set Acceleration error");
        m_bIsInitializing = false;
        return -1;
    }
    if(SetRobotDeceleration(m_dPvAcceleration) == -1)
    {
        ROS_ERROR("[MotorServoInit] init set Deceleration error");
        m_bIsInitializing = false;
        return -1;
    }

    UpdateOriginPosition();

    if(GpioControl(R_LIGHT_OFF) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn off red light failed");
        m_bIsInitializing = false;
        return -1;
    }

    if(GpioControl(G_LIGHT_ON) == -1)
    {
        ROS_ERROR("[MotorServoInit] turn on green light failed");
        m_bIsInitializing = false;
        return -1;
    }

    m_bIsMotorInit = true;
    m_bIsInitializing = false;

    PublishStatus(TASK_DONE);
    ROS_INFO("[MotorServoInit] motor init succeed");

    return 1;
}

/*************************************************
Function: CTrackRobot::SetMotorAcceleration
Description: 设置电机伺服的加速度 A
Input: float fVelocity 机器人运动减速度,单位 m/s^2
Output: void
Others: void
**************************************************/
int CTrackRobot::SetRobotAcceleration(double dAcceleration)
{
    CanFrame AccelerationCanData[2];
    CanFrame AccelerationCheckCanData[2];
    memset(AccelerationCanData, 0, sizeof(CanFrame)*2);
    memset(AccelerationCheckCanData, 0, sizeof(CanFrame)*2);

    int nCount = 0;
    m_MotorOne.SetAccelerationCmdPack(AccelerationCanData, dAcceleration);
    nCount = m_MotorOne.SetAccelerationCheckDataPack(AccelerationCheckCanData);

    if(SendCanFrame(AccelerationCanData, nCount, AccelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotAcceleration] set motor one Acceleration error");
        return -1;
    }

    memset(AccelerationCanData, 0, sizeof(CanFrame)*2);
    memset(AccelerationCheckCanData, 0, sizeof(CanFrame)*2);

    m_MotorTwo.SetAccelerationCmdPack(AccelerationCanData, dAcceleration);
    nCount = m_MotorTwo.SetAccelerationCheckDataPack(AccelerationCheckCanData);

    if(SendCanFrame(AccelerationCanData, nCount, AccelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotAcceleration] set motor two Acceleration error");
        return -1;
    }
    return 1;
}

/*************************************************
Function: CTrackRobot::SetMotorAcceleration
Description: 设置电机伺服的加速度 A
Input: float fVelocity 机器人运动减速度,单位 m/s^2
Output: void
Others: void
**************************************************/
int CTrackRobot::SetRobotDeceleration(double dDeceleration)
{
    CanFrame DecelerationCanData[2];
    CanFrame DecelerationCheckCanData[2];
    memset(DecelerationCanData, 0, sizeof(CanFrame)*2);
    memset(DecelerationCheckCanData, 0, sizeof(CanFrame)*2);

    int nCount = 0;
    m_MotorOne.SetDecelerationCmdPack(DecelerationCanData, dDeceleration);
    nCount = m_MotorOne.SetDecelerationCheckDataPack(DecelerationCheckCanData);

    if(SendCanFrame(DecelerationCanData, nCount, DecelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotDeceleration] set motor one Deceleration error");
        return -1;
    }

    memset(DecelerationCanData, 0, sizeof(CanFrame)*2);
    memset(DecelerationCheckCanData, 0, sizeof(CanFrame)*2);

    m_MotorTwo.SetDecelerationCmdPack(DecelerationCanData, dDeceleration);
    nCount = m_MotorTwo.SetDecelerationCheckDataPack(DecelerationCheckCanData);

    if(SendCanFrame(DecelerationCanData, nCount, DecelerationCheckCanData, true)== -1)
    {
        ROS_ERROR("[SetRobotDeceleration] set motor two Deceleration error");
        return -1;
    }
    return 1;
}

/*************************************************
Function: CTrackRobot::ForwardMotion
Description: 机器人向正方向运动
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ForwardMotion()
{
    m_nDirection = FORWARD;
    actual_tire_diameter = forward_diameter;

    if(m_nMode == PP)
    {
        if(m_nControlMode == TASK)
        {
            SetNewPos(m_dTargetDis);
        }
        else if(m_nControlMode == ALARM_LOCATION)
        {
            SetNewPos(m_dAlarmTargetDis);
        }
    }
    else if(m_nMode == PV)
    {
        m_bCheckAlarmData = false;
        m_dTargetSpeed = m_dTargetSpeedTemp;

        //m_dEndDis需要判断 处理目标距离无法完成抵达目标速度的正常匀加速和匀减速
        m_dEndDis = (m_dTargetSpeed*m_dTargetSpeed)/(2*m_dPvAcceleration) + END_DIS_AMEND;

        double dTargetDis = m_dTargetDis;
        if(m_nControlMode == ALARM_LOCATION)
            dTargetDis = m_dAlarmTargetDis;

        if(dTargetDis <= 2*m_dEndDis - END_DIS_AMEND)
        {
            m_dEndDis = dTargetDis*END_DIS_MODIFICATION;
        }

        ROS_DEBUG("[ForwardMotion] m_dEndDis :%f",m_dEndDis);
    }
    if(m_nAutoRunStatus == LOCATION_DONE)
    {
        m_nAutoRunStatus = INITIAL;
    }
}

/*************************************************
Function: CTrackRobot::BackwardMotion
Description: 机器人向正方向反向运动
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::BackwardMotion()
{
    m_nDirection = BACKWARD;

    actual_tire_diameter = back_diameter;

    if(m_nMode == PP)
    {
        if(m_nControlMode == TASK)
        {
            SetNewPos(-m_dTargetDis);
        }
        else if(m_nControlMode == ALARM_LOCATION)
        {
            SetNewPos(-m_dAlarmTargetDis);
        }
    }
    else if(m_nMode == PV)
    {
        m_bCheckAlarmData = false;
        m_dTargetSpeed = m_dTargetSpeedTemp;

        //m_dEndDis需要判断 处理目标距离无法完成抵达目标速度的正常匀加速和匀减速
        m_dEndDis = (m_dTargetSpeed*m_dTargetSpeed)/(2*m_dPvAcceleration) + END_DIS_AMEND;

        double dTargetDis = m_dTargetDis;
        if(m_nControlMode == ALARM_LOCATION)
            dTargetDis = m_dAlarmTargetDis;

        if(dTargetDis <= 2*m_dEndDis - END_DIS_AMEND)
        {
            m_dEndDis = dTargetDis*END_DIS_MODIFICATION;
        }

        ROS_DEBUG("[BackwardMotion] m_dEndDis :%f",m_dEndDis);
    }
    if(m_nAutoRunStatus == LOCATION_DONE)
    {
        m_nAutoRunStatus = INITIAL;
    }
}

/*************************************************
Function: CTrackRobot::StopMotion2
Description: 电机开始运动
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ReturnToOrigin(bool bDoneReturn)
{
    if(m_nReturnRecord)
    {
        m_dDisTemp = 0xFFFFFFFFFFFFFFFF;
    }
    if(m_nDirection == FORWARD || m_nDirection == BACKWARD_RETURN)
    {
        actual_tire_diameter = forward_diameter;
    }
    else if(m_nDirection == BACKWARD || m_nDirection == FORWARD_RETURN)
    {
        actual_tire_diameter = back_diameter;
    }

    if(m_nMode == PP)
    {
        double dTargetDis = m_dTargetDis;
        if(m_nControlMode != TASK)
        {
            int nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_RelocationMotorPos.nMotorTwoPos;

            dTargetDis = (nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;

            if(dTargetDis < 0)
            {
                dTargetDis = -dTargetDis;
                m_nDirection = BACKWARD;
            }
            else
            {
                m_nDirection = FORWARD;
            }
        }

        dTargetDis = abs(dTargetDis);

        if(dTargetDis <= 0.002 || abs(m_dDistance) <= 0.002)
        {
            ROS_DEBUG("[ReturnToOrigin] Robot is already at the origin,return,dTargetDis=%f,m_dDistance=%f",dTargetDis,m_dDistance);
            return;
        }

        if(m_nDirection == FORWARD || m_nDirection == FORWARD_RETURN)
        {
            ROS_DEBUG("[ReturnToOrigin] BackwardMotion");
            m_nDirection = FORWARD_RETURN;
            SetNewPos(-dTargetDis);
        }
        else if(m_nDirection == BACKWARD || m_nDirection == BACKWARD_RETURN)
        {
            ROS_DEBUG("[ReturnToOrigin] ForwardMotion");
            m_nDirection = BACKWARD_RETURN;
            SetNewPos(dTargetDis);
        }
    }
    else if(m_nMode == PV)
    {
        double dTargetDis = m_dTargetDis;
        if(m_nControlMode != TASK)
        {
            int nMotor_One_Displacement = m_CurrentMotorPos.nMotorOnePos-m_RelocationMotorPos.nMotorOnePos;
            int nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_RelocationMotorPos.nMotorTwoPos;

            m_MotorPos.nMotorOnePos = m_RelocationMotorPos.nMotorOnePos;
            m_MotorPos.nMotorTwoPos = m_RelocationMotorPos.nMotorTwoPos;

            double dMotorOneDis = (nMotor_One_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
            dTargetDis = ((nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000 + dMotorOneDis)/2;

            if(dTargetDis < 0)
            {
                dTargetDis = -dTargetDis;
                m_nDirection = BACKWARD;
            }
            else
            {
                m_nDirection = FORWARD;
            }
            m_dDistance = dTargetDis;
        }

        if(abs(m_dDistance) <= 0.002)
        {
            ROS_DEBUG("[ReturnToOrigin] Robot is already at the origin,return");
            return;
        }

        m_dEndDis = (m_dTargetSpeedTemp*m_dTargetSpeedTemp)/(2*m_dPvAcceleration) + END_DIS_AMEND;
        if(dTargetDis <= 2*m_dEndDis - END_DIS_AMEND)
        {
            m_dEndDis = dTargetDis*END_DIS_MODIFICATION;
        }

//        ROS_DEBUG("[ReturnToOrigin] m_dEndDis :c%f",m_dEndDis);

        if(m_nDirection == FORWARD || m_nDirection == FORWARD_RETURN)
        {
            ROS_DEBUG("[ReturnToOrigin] BackwardMotion");
            m_dTargetSpeed = -abs(m_dTargetSpeedTemp);
            m_nDirection = FORWARD_RETURN;

            if(m_nControlMode == TASK && bDoneReturn)
            {
                m_MotorPos.nMotorOnePos = m_CurrentMotorPos.nMotorOnePos - m_MotorOne.TransMileToPulse(m_dTargetDis);
                m_MotorPos.nMotorTwoPos = m_CurrentMotorPos.nMotorTwoPos - m_MotorTwo.TransMileToPulse(m_dTargetDis);
            }
        }
        else if(m_nDirection == BACKWARD || m_nDirection == BACKWARD_RETURN)
        {
            ROS_DEBUG("[ReturnToOrigin] ForwardMotion");
            m_dTargetSpeed = abs(m_dTargetSpeedTemp);
            m_nDirection = BACKWARD_RETURN;

            if(m_nControlMode == TASK && bDoneReturn)
            {
                m_MotorPos.nMotorOnePos = m_CurrentMotorPos.nMotorOnePos + m_MotorOne.TransMileToPulse(m_dTargetDis);
                m_MotorPos.nMotorTwoPos = m_CurrentMotorPos.nMotorTwoPos + m_MotorTwo.TransMileToPulse(m_dTargetDis);
            }
        }

        if(m_bCheckAlarmData)
        {
            m_bCheckAlarmData = false;
        }
    }
}

/*************************************************
Function: CTrackRobot::MotorSpeedUp
Description: 电机运动加速
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MotorSpeedUp()
{
    std::unique_lock<std::mutex> Lock(m_CurrentSpeedMutex);
    if(m_nMode == PP)
    {
        m_dTargetSpeed += SPEED_INCREMENT;
    }
    else if(m_nMode == PV)
    {
        if(m_nDirection == FORWARD)
        {
            m_dTargetSpeed += SPEED_INCREMENT;
        }
        else
        {
            m_dTargetSpeed -= SPEED_INCREMENT;
        }
    }
    Lock.unlock();
}

/*************************************************
Function: CTrackRobot::MotorSpeedDown
Description: 电机运动减速
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MotorSpeedDown()
{
    std::unique_lock<std::mutex> Lock(m_CurrentSpeedMutex);
    if(m_nMode == PP)
    {
        m_dTargetSpeed -= SPEED_INCREMENT;
        if(m_dTargetSpeed <= 0)
            m_dTargetSpeed = 0;

    }
    else if(m_nMode == PV)
    {
        if(m_nDirection == FORWARD)
        {
            m_dTargetSpeed -= SPEED_INCREMENT;
            if(m_dTargetSpeed <= 0)
            {
                m_dTargetSpeed = 0;
            }
        }
        else
        {
            m_dTargetSpeed -= SPEED_INCREMENT;
            if(m_dTargetSpeed >= 0)
            {
                m_dTargetSpeed = 0;
            }
        }
    }
    Lock.unlock();
}

/*************************************************
Function: CTrackRobot::SetMotorSpeed
Description: 设置电机运动速度
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SetMotorSpeed(double dSpeed)
{
    CanFrame SpeedCanData[2];
    CanFrame *pCanData = SpeedCanData;
    int nCount = 0;

    if(m_nMode == PV)
    {
        nCount += m_MotorOne.SetSpeedPdoCmdPack(pCanData, dSpeed);
        pCanData = SpeedCanData + nCount;
    }

    nCount += m_MotorTwo.SetSpeedPdoCmdPack(pCanData, dSpeed);

    SendCanFrame(SpeedCanData, nCount);
}

/*************************************************
Function: CTrackRobot::SendCmdOfGetMotorTemp
Description: 发送获取电机伺服温度报文
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCmdOfGetMotorTemp()
{
    CanFrame CanData[2];
    CanFrame *pCanData = CanData;
    int nCount = 0;
    nCount += m_MotorOne.GetMotorTempPack(pCanData);
    pCanData = CanData + nCount;
    nCount += m_MotorTwo.GetMotorTempPack(pCanData);

    SendCanFrame(CanData, nCount);
}

/*************************************************
Function: CTrackRobot::SendCmdOfGetMotorErrorCode
Description: 发送获取电机伺服温度报文
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCmdOfGetMotorErrorCode()
{
    CanFrame CanData[2];
    CanFrame *pCanData = CanData;
    int nCount = 0;
    nCount += m_MotorOne.GetErrorCodeCmdPack(pCanData);
    pCanData = CanData + nCount;
    nCount += m_MotorTwo.GetErrorCodeCmdPack(pCanData);

    SendCanFrame(CanData, nCount);
}

/*************************************************
Function: CTrackRobot::SendHeartBeatCanFrame
Description: 设置电机运动速度
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendHeartBeatCanFrame()
{
    SendCanFrame(MASTER_HEART_BEAT_ID, 0, 0);
}

/*************************************************
Function: CTrackRobot::SetMotorSpeed
Description: 设置电机运动速度
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCmdOfGetBatteryInfo()
{
    SendCanFrame(0x16, 8, 0x16BB00000000007E);
    SendCanFrame(0x16, 8, 0x16BE00000000007E);
}

/*************************************************
Function: CTrackRobot::SetNewPos
Description: 位置模式下,设置电机的新的位置点
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SetNewPos(double dDistance)
{
    CanFrame PosCanData[5];
    CanFrame *pCanData = PosCanData;

    int nType = RELATIVE;

    int nCount = m_MotorTwo.SetNewPosCmdPack(pCanData, nType, dDistance);

    pCanData = PosCanData + nCount;
    m_MotorOne.SetSdoCanFrame(pCanData, SdoControlWords[ENABLE_OPERATION]);
    nCount ++;

    SendCanFrame(PosCanData, nCount);
}

/*************************************************
Function: CTrackRobot::QuickStop
Description: 电机运动停止，急停
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::QuickStop()
{
    m_dTargetSpeed = 0;

    CanFrame StopCanData[2];
    CanFrame *pCanData = StopCanData;
    int nCount = 0;
    nCount += m_MotorOne.QuickStopCmdPack(pCanData);
    pCanData = StopCanData + nCount;
    nCount += m_MotorTwo.QuickStopCmdPack(pCanData);

    SendCanFrame(StopCanData, nCount);
//    ROS_DEBUG("[QuickStop] send quick stop cmd");
}

/*************************************************
Function: CTrackRobot::StopMotion
Description: 电机运动停止，按照设置的匀减速，速度减至0
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::StopMotion(bool bJoy)
{
    if(m_nMode == PV)
    {
        m_dTargetSpeed = 0;
    }
    else if(m_nMode == PP)
    {
        CanFrame StopCanData[2];
        CanFrame *pCanData = StopCanData;
        int nCount = 0;
        nCount += m_MotorOne.StopCmdPack(pCanData);
        pCanData = StopCanData + nCount;
        nCount += m_MotorTwo.StopCmdPack(pCanData);

        SendCanFrame(StopCanData, nCount);
    }
    if(!bJoy)
        m_bIsMotorPause = true;
}

/*************************************************
Function: CTrackRobot::ResumeMotion
Description: 电机暂停后继续执行动作
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ResumeMotion()
{
    if(m_bIsMotorPause)
    {
        ROS_DEBUG("[ResumeMotion] start cmd.");

        double dRemainDis = 0.0;

        if(m_nDirection >= FORWARD_RETURN)
            dRemainDis = m_dDistance;
        else if(m_nDirection <= BACKWARD)
            dRemainDis = m_dTargetDis - m_dDistance;

        if(m_nMode == PV)
        {
            //m_dEndDis需要判断 处理目标距离无法完成抵达目标速度的正常匀加速和匀减速
            if(dRemainDis <= 2*m_dEndDis - END_DIS_AMEND)
            {
                m_dEndDis = dRemainDis*END_DIS_MODIFICATION;
            }
//            ROS_DEBUG("[ResumeMotion] start m_dEndDis :%f",m_dEndDis);

            m_dTargetSpeed = (m_nDirection == FORWARD || m_nDirection == BACKWARD_RETURN) ? abs(m_dTargetSpeedTemp):-abs(m_dTargetSpeedTemp);
            ROS_DEBUG("[ResumeMotion] start m_dTargetSpeed=%f,m_dTargetSpeedTemp=%f",m_dTargetSpeed,m_dTargetSpeedTemp);
        }
        else if(m_nMode == PP)
        {
            double dNewTargetDis = (m_nDirection == FORWARD || m_nDirection == BACKWARD_RETURN) ? abs(dRemainDis):-abs(dRemainDis);
            SetNewPos(dNewTargetDis);
        }
        m_bIsMotorPause = false;
    }
    else
    {
        ROS_DEBUG("[ResumeMotion] did not receive stop cmd.");
    }
}

/*************************************************
Function: CTrackRobot::MotorPowerOff
Description: 电机伺服控制掉电
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::MotorPowerOff()
{
    SendCanFrame(0x601, 8, 0x2B40600006000000);//断电
    SendCanFrame(0x602, 8, 0x2B40600006000000);
    m_bMotorRunning = false;
}

/*************************************************
Function: CTrackRobot::ClearWarn
Description: 清除伺服警告
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ClearWarn()
{
    if(m_bIsMotorInit)
    {
        ROS_INFO("[ClearWarn]m_bIsMotorInit = %d, return.",m_bIsMotorInit);
        return;
    }
    int nSize = 2;
    CanFrame ClearCanData[nSize];
    CanFrame ClearCanCheckData[nSize];
    bool bIsCheck = true;

    memset(ClearCanData, 0, sizeof(CanFrame)*nSize);
    memset(ClearCanCheckData, 0, sizeof(CanFrame)*nSize);

    m_MotorOne.ClearWarnPack(ClearCanData);
    int nCount = m_MotorOne.ClearWarnCheckCmdPack(ClearCanCheckData);

    if(SendCanFrame(ClearCanData, nCount, ClearCanCheckData, bIsCheck) == -1)
    {
        ROS_ERROR("[ClearWarn] motor one clear warn error");
    }

    memset(ClearCanData, 0, sizeof(CanFrame)*nSize);
    memset(ClearCanCheckData, 0, sizeof(CanFrame)*nSize);


    m_MotorTwo.ClearWarnPack(ClearCanData);
    nCount = m_MotorTwo.ClearWarnCheckCmdPack(ClearCanCheckData);

    if(SendCanFrame(ClearCanData, nCount, ClearCanCheckData, bIsCheck) == -1)
    {
        ROS_ERROR("[ClearWarn] motor two clear warn error");
    }

    m_sMotorOneStatus = "normal";
    m_sMotorTwoStatus = "normal";
}

/*************************************************
Function: CTrackRobot::SendGetMotorPosCmd
Description: 获取电机的当前位置值,绝对位移
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendGetMotorInfoCmd()
{
    SendCanFrame(GET_MOTOR_INFO_PDO_ID, 0, 0);
}

/*************************************************
Function: CTrackRobot::SendCanFrame
Description: can数据发送函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::SendCanFrame(unsigned ID, unsigned char DataLen, ULL frameData)
{
    CanFrame canFrame;

    UsbCan.SetCanFrame(&canFrame, ID, DataLen, frameData);

    std::unique_lock<std::mutex> LockSendCanFrame(m_CanTXMutex);
    m_dCanTXDeque.push_back(canFrame);
    m_CanTXCondition.notify_one();
}

int CTrackRobot::SendCanFrame(CanFrame *sendCanData, int nCount, CanFrame *checkCanData, bool bIsCheck)
{
    if(bIsCheck)
    {
        std::unique_lock<std::mutex> Lock(m_SdoCheckMutex);
        m_SdoCheckBuf = new CanFrame[nCount];
        memcpy(m_SdoCheckBuf, checkCanData, sizeof(CanFrame)*nCount);
        Lock.unlock();

        m_nSdoCheckCount = 0;
        m_bCheckSdo = true;
    }

    std::unique_lock<std::mutex> LockSendCanFrame(m_CanTXMutex);
    for(int i=0; i<nCount; i++)
    {
        m_dCanTXDeque.push_back(sendCanData[i]);
    }
    m_CanTXCondition.notify_one();
    LockSendCanFrame.unlock();

    if(bIsCheck)
    {
        timespec StartTime;
        timespec_get(&StartTime, TIME_UTC);

        while(m_nSdoCheckCount < nCount)
        {
            this_thread::sleep_for(std::chrono::milliseconds(10));

            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
            if(pollInterval > (nCount * 20) || !m_bCheckSdo)
            {
                if(m_bCheckSdo)
                    ROS_WARN("[SendCanFrame] receive data time out");
                m_nSdoCheckCount = 0;
                m_bCheckSdo = false;
                return -1;
            }
        }

        std::unique_lock<std::mutex> Lock1(m_SdoCheckMutex);
        delete m_SdoCheckBuf;
        m_SdoCheckBuf = nullptr;
        Lock1.unlock();

        m_nSdoCheckCount = 0;
        m_bCheckSdo = false;
    }
    return 1;
}

/*************************************************
Function: CTrackRobot::PublishStatus
Description: can帧数据打印函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::PublishStatus(int nStatus)
{
    custom_msgs::TrainRobotControlAck status;

    status.sender = "track_robot";
    status.status = vsRobotStatus[nStatus];
    status.header.stamp = ros::Time::now();
    status.header.frame_id = "status";

    m_StatusPublisher.publish(status);
}

/*************************************************
Function: CTrackRobot::PublishStatus
Description: can帧数据打印函数
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::PublishStatus(string &sStatus)
{
    custom_msgs::TrainRobotControlAck status;

    status.sender = "track_robot";
    status.status = sStatus;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = "status";

    m_StatusPublisher.publish(status);
}

/*************************************************
Function: CTrackRobot::CommandCallBack
Description: 后台控制回调函数
Input: const custom_msgs::TrainRobotControl::ConstPtr &Command, 后台控制消息
Output: void
Others: void
**************************************************/
void CTrackRobot::CommandCallBack(const custom_msgs::TrainRobotControl::ConstPtr &Command)
{
    custom_msgs::TrainRobotControlAck status;

    if(!m_bIsMotorInit)
    {
        ROS_WARN("[CommandCallBack] motor is not init.");
        return;
    }
    if(m_bIsEStopButtonPressed)
    {
        ROS_WARN("[CommandCallBack] E-Stop button is pressed.");
        return;
    }

    if(Command->cmd == "joy_run")
    {
        if(m_bMotorRunning && m_nControlMode != JOY)
        {
            ROS_WARN("[CommandCallBack] can not use joy,please wait for a moment");
            return;
        }
        m_nControlMode = JOY;

        std::unique_lock<std::mutex> lock(m_JoyTimeOutCheckMutex);
        if(m_tTimeOfLastJoyCmd.tv_sec == 0 && m_tTimeOfLastJoyCmd.tv_nsec == 0)
        {
            m_dTargetSpeed = Command->speed;
            if(m_nMode == PP)
            {
                SetMotorSpeed(abs(m_dTargetSpeed));
                if(Command->speed > 0)
                    SetNewPos(2000.0);
                else
                    SetNewPos(-2000.0);
            }
            m_nDirection = m_dTargetSpeed > 0 ? FORWARD : BACKWARD;
        }
        timespec_get(&m_tTimeOfLastJoyCmd, TIME_UTC);
        lock.unlock();
    }
    else if(Command->cmd == "run" || Command->cmd == "alarm_run")
    {
        if(m_bMotorRunning)
        {
            ROS_WARN("[CommandCallBack] robot is running, return.");
            return;
        }
        if(m_bIsMotorPause)
        {
            ROS_WARN("[CommandCallBack] robot is paused,please use recovery, return.");
            return;
        }
        if(Command->cmd == "alarm_run")
        {
            m_nControlMode = ALARM_LOCATION;

            if(m_dLimitDis != Command->limit_dis/1000)
            {
                ROS_DEBUG("[CommandCallBack] alarm run set m_dLimitDis :%f",Command->limit_dis/1000);
                m_dLimitDis = Command->limit_dis/1000;
            }

            if(Command->distance != m_dAlarmTargetDis)
            {
                ROS_DEBUG("[CommandCallBack] alarm run set m_dAlarmTargetDis :%f",Command->distance);
                m_dAlarmTargetDis = Command->distance;
            }
        }
        else
        {
            m_nControlMode = TASK;
            m_dDisTemp = 0xFFFFFFFFFFFFFFFF;
            m_bForwardRelocation = (Command->speed > 0);

            if(m_nUseAutoRun != Command->back)
            {
                ROS_INFO("[CommandCallBack] set m_nUseAutoRun :%d",Command->back);
                m_nUseAutoRun = Command->back;
            }
            if(Command->distance != m_dTargetDis)
            {
                ROS_INFO("[CommandCallBack] set distance :%f",Command->distance);
                m_dTargetDis = Command->distance;
            }
        }

        UpdateOriginPosition();

        if(Command->speed !=  m_dTargetSpeed)
        {
            ROS_INFO("[CommandCallBack] set speed :%f",Command->speed);
            m_dTargetSpeedTemp = Command->speed;

            if(m_nMode == PP)
            {
                SetMotorSpeed(abs(m_dTargetSpeedTemp));
            }
            if(m_dTargetSpeedTemp > 0)
            {
                if(Command->cmd == "alarm_run")
                    ROS_INFO("[CommandCallBack] alarm run forward motion start");
                else
                    ROS_INFO("[CommandCallBack] forward motion start");

                ForwardMotion();
                PublishStatus(RUN_FORWARD);
            }
            else
            {
                if(Command->cmd == "alarm_run")
                    ROS_INFO("[CommandCallBack] alarm run backward motion start");
                else
                    ROS_INFO("[CommandCallBack] backward motion start");

                BackwardMotion();
                PublishStatus(RUN_BACKWARD);
            }
        }
    }
    else if(Command->cmd == "stop")
    {
        PublishStatus(STOP);
        ROS_INFO("[CommandCallBack] stop cmd.");
        if(!m_bMotorRunning)
        {
            ROS_WARN("[CommandCallBack] robot has stopped, return.");
        }
        else if(m_bIsSlowing)
        {
            m_bIsSlowing = false;
            ROS_WARN("[CommandCallBack] motor is slowing, return.");
        }
        else
        {
            StopMotion();
        }
    }
    else if(Command->cmd == "start")
    {
        PublishStatus(START);
        ResumeMotion();
    }
    else if(Command->cmd == "return")
    {
        PublishStatus(RETURN);

        if(m_bIsMotorPause)
            m_bIsMotorPause = false;

        if(m_bMotorRunning)
        {
            ROS_WARN("[CommandCallBack] robot is running, return.");
            return;
        }
        ROS_INFO("[CommandCallBack] return to origin.");
        if(Command->speed !=  m_dTargetSpeed)
        {
            ROS_INFO("[CommandCallBack] return set speed :%f",Command->speed);
            m_dTargetSpeedTemp = Command->speed;
        }
        ReturnToOrigin();
    }
    //added by btrmg for adjust mileage 2020.01.07
    else if(Command->cmd =="adjust_mileage")
    {
        ROS_INFO("[CommandCallBack] adjust expected mileage:%d and actual mileage:%d",Command->expected_mileage,Command->actual_mileage);
        if(0 != Command->actual_mileage && 0 != Command->expected_mileage)
        {
            if(Command->speed > 0)
            {
                forward_diameter = forward_diameter/Command->expected_mileage*Command->actual_mileage;
                string str_diameter;
                str_diameter = std::to_string(forward_diameter);
                updata_XML("forward_diameter",str_diameter);
                ROS_DEBUG("[CommandCallBack] adjusted forward diamete is:%f",forward_diameter);
            }
            else if(Command->speed < 0)
            {
                back_diameter = back_diameter/Command->expected_mileage*Command->actual_mileage;
                string str_diameter;
                str_diameter = std::to_string(back_diameter);
                updata_XML("back_diameter",str_diameter);
                ROS_DEBUG("[CommandCallBack] adjusted back diameter is :%f",back_diameter);
            }

        }
    }
    //added end
}

/*************************************************
Function: CTrackRobot::JoystickCallBack
Description: 手柄控制回调函数
Input: custom_msgs::TrainJoyControlCmd::ConstPtr &JoyCommand , 手柄控制消息
Output: void
Others: void
**************************************************/
void CTrackRobot::JoystickCallBack(const custom_msgs::TrainJoyControlCmd::ConstPtr &JoyCommand)
{

}

/*************************************************
Function: CTrackRobot::OdomCallBack
Description: 滤波后的历程消息
Input: nav_msgs::Odometry::ConstPtr &OdomMsg , 滤波后的里程消息
Output: void
Others: void
**************************************************/
void CTrackRobot::OdomCallBack(const nav_msgs::Odometry::ConstPtr &OdomMsg)
{
    m_dCurrentOdom = OdomMsg->pose.pose.position.x;
    m_dDistance = m_dCurrentOdom - m_dOdom;

    if(m_nDirection == BACKWARD || m_nDirection == BACKWARD_RETURN)
    {
        m_dDistance =  m_dOdom - m_dCurrentOdom;
    }
    if(m_bIsUpdateOrign)
    {
        m_dOdom = m_dCurrentOdom;
        m_bIsUpdateOrign = false;
    }
}

/*************************************************
Function: CTrackRobot::BatteryServiceFunc
Description: 电池状态查询服务端
Input: custom_msgs::BatteryStatus::Request &Req 请求
        custom_msgs::BatteryStatus::Response &Resp 响应
Output: true or false
Others: void
**************************************************/
bool CTrackRobot::BatteryServiceFunc(custom_msgs::BatteryStatus::Request &Req, custom_msgs::BatteryStatus::Response &Resp)
{
    Resp.response = Req.request + "_response";

    SendCanFrame(0x16, 8, 0x16BB00000000007E);

    timespec StartTime;
    timespec_get(&StartTime, TIME_UTC);
    this_thread::sleep_for(std::chrono::milliseconds(20));

    while(ros::ok())
    {
        //此处加锁是规避多个client高并发请求时引入的数据混乱
        std::unique_lock<std::mutex> lock(m_ButteryStatusMutex);
        int nButteryPower = m_nBatteryPower;
        lock.unlock();
        if(nButteryPower)
        {
            Resp.data = "{";
            Resp.data += "\"battery_power\":" + to_string(nButteryPower);
            Resp.data += "}";

            std::unique_lock<std::mutex> locker(m_ButteryStatusMutex);
            m_nBatteryPower = 0;
            locker.unlock();

            return true;
        }
        else
        {
            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
            if(pollInterval > 1000)
            {
                Resp.data = "timeout error!";
                return true;
            }
            SendCanFrame(0x16, 8, 0x16BB00000000007E);
            this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

/*************************************************
Function: CTrackRobot::CurrentPosServiceFunc
Description: 获取当前位置信息
Input: custom_msgs::CurrentPosition::Request &Req 请求
        custom_msgs::CurrentPosition::Response &Resp 响应
Output: true or false
Others: void
**************************************************/
bool CTrackRobot::CurrentPosServiceFunc(custom_msgs::CurrentPosition::Request &Req, custom_msgs::CurrentPosition::Response &Resp)
{
    Resp.response = Req.request + "_response";

    std::unique_lock<std::mutex> locker(m_CurrentPosScanBufMutex);
    if(!m_bGetAxesData && !m_dCurrentPosScanBuf.empty())
    {
        m_dCurrentPosScanBuf.clear();
//        ROS_DEBUG("[CurrentPosServiceFunc]m_dCurrentPosScanBuf.clear()");
    }
    locker.unlock();

    m_bGetAxesData = true;

    std::deque<sScanBuf> dScanBufTemp;

    timespec StartTime;
    timespec_get(&StartTime, TIME_UTC);
    this_thread::sleep_for(std::chrono::milliseconds(20));

    while(ros::ok())
    {
        dScanBufTemp.clear();
        std::unique_lock<std::mutex> lock(m_CurrentPosScanBufMutex);
        dScanBufTemp.assign(m_dCurrentPosScanBuf.begin(), m_dCurrentPosScanBuf.end());
        lock.unlock();

        if(dScanBufTemp.size() >= CURRENT_DATA_TOTAL_NUM)
        {
            m_bGetAxesData = false;
            break;
        }
        else
        {
            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
            if(pollInterval > 1000)
            {
                ROS_ERROR("[CurrentPosServiceFunc] timeout error");
                Resp.data = "timeout error!";
                m_bGetAxesData = false;
                return true;
            }

            this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    //二维数组数据
    float fScanMatrix[CURRENT_DATA_TOTAL_NUM][LASER_MAX_DATA_LEN]={0};
    sScanBuf SumRanges;
    memset(&SumRanges, 0 , sizeof(sScanBuf));
    //将激光数据copy至二维数组当中
    for(int row=0; row < CURRENT_DATA_TOTAL_NUM; row++)
    {
        sScanBuf sBufTemp = dScanBufTemp[row];

        for(int column=0; column < LASER_MAX_DATA_LEN; column++)
        {
            fScanMatrix[row][column] = sBufTemp.fRanges[column];
        }
        SumRanges.x = sBufTemp.x;
    }
    //剔除同一角度位置的有0出现的数据,将没有0出现的数据累计求和
    for(int column=0; column<LASER_MAX_DATA_LEN; column++)
    {
        for(int row=0; row<CURRENT_DATA_TOTAL_NUM; row++)
        {
            if(fScanMatrix[row][column] == 0)
            {
                SumRanges.fRanges[column] = 0;
                break;
            }
            else
            {
                SumRanges.fRanges[column] += fScanMatrix[row][column];
            }
        }
    }
    //求均值
    for(int i=0; i<LASER_MAX_DATA_LEN; i++)
    {
        SumRanges.fRanges[i] = SumRanges.fRanges[i]/CURRENT_DATA_TOTAL_NUM;
    }

    SumRanges.ullRangeSize = LASER_MAX_DATA_LEN;

    sPositionData desPositionData = {0};
    int nDataLen = 0;

    nDataLen = TransPosBufData(desPositionData, SumRanges);
    Resp.x =  desPositionData.x;

    Resp.y.resize((unsigned long)nDataLen);
    Resp.z.resize((unsigned long)nDataLen);

    for(int i = 0; i < nDataLen; i++)
    {
        Resp.y[i] = desPositionData.y[i];
        Resp.z[i] = desPositionData.z[i];
    }

    Resp.data = "succeed";
    return true;
}

/*************************************************
Function: CTrackRobot::ScanCallBack
Description: 激光节点消息回调函数,按照激光的回调频率对里程x做平均差值处理，
             并把处理后的x及激光原始数据缓存到队列当中
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &scanData)
{
    //以下情况直接返回：
    // 前端没有获取当前数据 并且 ①机器人停止状态 ②机器人做返程运动并且不记录返程数据 ③机器人控制模式为手柄模式
    if(!m_bGetAxesData)
    {
        if(!m_bMotorRunning || (m_nDirection >= FORWARD_RETURN && !m_nReturnRecord) || m_nControlMode == JOY)
        {
            if(!m_dScanSrcBufDeque.empty())
            {
                std::unique_lock<std::mutex> lock(m_ScanSrcBufMutex);
                m_dScanSrcBufDeque.clear();
                lock.unlock();
            }
            return;
        }
    }
    ULL ullRangeSize = scanData->ranges.size();
    sScanBuf ScanBuf;
    memset(&ScanBuf, 0 , sizeof(sScanBuf));
    memcpy(&(ScanBuf.fRanges[0]), &(scanData->ranges[0]), sizeof(ScanBuf.fRanges[0])*ullRangeSize);

    ScanBuf.x = m_dDistance;
    ScanBuf.ullRangeSize = ullRangeSize;

    //前台获取当前位置数据的service用
    if(m_bGetAxesData)
    {
        //按键获取当前位置信息（x y z）
        std::unique_lock<std::mutex> locker(m_CurrentPosScanBufMutex);
        m_dCurrentPosScanBuf.push_back(ScanBuf);
        if(m_dCurrentPosScanBuf.size() > CURRENT_DATA_TOTAL_NUM)
        {
            m_dCurrentPosScanBuf.pop_front();
        }
        locker.unlock();

        if(!m_bMotorRunning || (m_nDirection >= FORWARD_RETURN && !m_nReturnRecord) || m_nControlMode == JOY)
            return;
    }

    std::unique_lock<std::mutex> lock(m_ScanSrcBufMutex);
    m_dScanSrcBufDeque.push_back(ScanBuf);
    if(m_dScanSrcBufDeque.size() > CURRENT_DATA_TOTAL_NUM)
    {
        m_dScanSrcBufDeque.pop_front();
    }
    lock.unlock();
}

/*************************************************
Function: CTrackRobot::ImuCallBack
Description: 激光节点消息回调函数,按照激光的回调频率对里程x做平均差值处理，
             并把处理后的x及激光原始数据缓存到队列当中
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ImuCallBack(const sensor_msgs::Imu::ConstPtr &ImuData)
{
    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(tf::Quaternion(ImuData->orientation.x,
                                 ImuData->orientation.y,
                                 ImuData->orientation.z,
                                 ImuData->orientation.w)).getRPY(dRoll, dPitch, dYaw);
//    ROS_INFO("[ImuCallBack] dRoll=%f, dPitch=%f, dYaw=%f",dRoll/M_PI*180, dPitch/M_PI*180, dYaw/M_PI*180);

    m_dPitchAngle = 180*dPitch/M_PI;
    m_dRealScanTitle = m_dScanTitle - m_dPitchAngle;
    m_dInitialAngle = 180.0-m_dRealScanTitle-(90.0-LASER_ANGLE_RANGE/2)-LASER_EXCISION_ANGLE ;
}


/*************************************************
Function: CTrackRobot::ScanCallBack
Description: 激光节点消息回调函数,按照激光的回调频率对里程x做平均差值处理，
             并把处理后的x及激光原始数据缓存到队列当中
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ScanThreadFunc()
{
    asr_sick_lms_400 lms_ = asr_sick_lms_400 ("192.168.1.100", 2111, 0);

    // Attempt to connect to the laser unit
    if (lms_.Connect () != 0)
    {
        ROS_ERROR("[ScanThreadFunc] Connecting to SICK LMS4000 on 192.168.1.100:2111 filed");
    }
    ROS_INFO ("[ScanThreadFunc] Connecting to SICK LMS4000 on 192.168.1.100:2111 succeed");

    // Stop the measurement process (in case it's running from another instance)
    lms_.StopMeasurement();


    // Login to userlevel 4
    if (lms_.SetUserLevel (4, "81BE23AA") != 0)
    {
        ROS_ERROR("[ScanThreadFunc] Unable to change user level to 'Service' using ");
    }
    else
    {
        ROS_INFO("[ScanThreadFunc]Set User Level to 'Service'");
    }

    // Start Continous measurements
    lms_.StartMeasurement (true);
    lms_.sMNRUN ();
    lms_.LMDscandata (1);
    ROS_INFO("[ScanThreadFunc]start read measurement data");

    while (ros::ok ())
    {
        sensor_msgs::LaserScan scanData = lms_.ReadMeasurement();
        //以下情况直接返回：
        // 前端没有获取当前数据 并且 ①机器人停止状态 ②机器人做返程运动并且不记录返程数据 ③机器人控制模式为手柄模式
        if(!m_bGetAxesData)
        {
            if(!m_bMotorRunning || (m_nDirection >= FORWARD_RETURN && !m_nReturnRecord) || m_nControlMode == JOY)
            {
                if(!m_dScanSrcBufDeque.empty())
                {
                    std::unique_lock<std::mutex> lock(m_ScanSrcBufMutex);
                    m_dScanSrcBufDeque.clear();
                    lock.unlock();
                }
//                this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }
//        sensor_msgs::LaserScan scanData = lms_.LMDscandataOneTimes();

        if (scanData.ranges.empty())
            continue;

        double dDistance, d_x;
        ULL ullRangeSize = scanData.ranges.size();

        sScanBuf ScanBuf;
        memset(&ScanBuf, 0 , sizeof(sScanBuf));
        memcpy(&(ScanBuf.fRanges[0]), &(scanData.ranges[0]), sizeof(ScanBuf.fRanges[0])*ullRangeSize);

        dDistance = m_dDistance;
        ScanBuf.x = dDistance;
        ScanBuf.ullRangeSize = ullRangeSize;
//        ROS_INFO("[ScanThreadFunc] get scan data");

        //前台获取当前位置数据的service用
        if(m_bGetAxesData)
        {
            //按键获取当前位置信息（x y z）
            std::unique_lock<std::mutex> locker(m_CurrentPosScanBufMutex);
            m_dCurrentPosScanBuf.push_back(ScanBuf);
            if(m_dCurrentPosScanBuf.size() > CURRENT_DATA_TOTAL_NUM)
            {
                m_dCurrentPosScanBuf.pop_front();
            }
            locker.unlock();

            if(!m_bMotorRunning || (m_nDirection >= FORWARD_RETURN && !m_nReturnRecord) || m_nControlMode == JOY)
                continue;
        }

        m_dScanSrcBufDeque.push_back(ScanBuf);

//涉及里程的平均插值，第一个位置数据单独处理（直接保存）
        if(m_dDisTemp == 0xFFFFFFFFFFFFFFFF)
        {
            std::unique_lock<std::mutex> lock(m_ScanManBufMutex);
            m_dScanManBufDeque.push_back(ScanBuf);
            lock.unlock();
            m_ScanBufCondition.notify_one();

            m_dScanSrcBufDeque.clear();
            m_dDisTemp = dDistance;
            continue;
        }
//对x做平均插值
        if(abs(dDistance - m_dDisTemp) > 0.0001)
        {
            ULL ullRangeCount =  m_dScanSrcBufDeque.size();
//        ROS_DEBUG("dDistance=%f,m_dDisTemp=%f,ullRangeCount=%lld",dDistance,m_dDisTemp,ullRangeCount);

            std::unique_lock<std::mutex> lock(m_ScanManBufMutex);

            for(int i=0; i<ullRangeCount; i++)
            {
                sScanBuf ScanBufTemp = m_dScanSrcBufDeque[i];
                if(ullRangeCount == 1)
                {
                    d_x = dDistance;
                }
                else
                {
                    d_x = m_dDisTemp + ((dDistance-m_dDisTemp)/ullRangeCount)*(i+1);
                }

                if(abs(d_x - m_dDisTemp) < 0.0001)
                    continue;

                ScanBufTemp.x = d_x;

//            ROS_DEBUG("d_x=%f",d_x);

                m_dScanManBufDeque.push_back(ScanBufTemp);
            }
            lock.unlock();
            m_dScanSrcBufDeque.clear();
            m_ScanBufCondition.notify_one();
            m_dDisTemp = dDistance;
        }
    }
    lms_.StopMeasurement ();
}

/*************************************************
Function: CTrackRobot::ManageScanDataThreadFunc
Description:数据过滤线程功能函数，保证两组数据中x的差值大于0.0001米
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::ManageScanDataThreadFunc()
{
    unsigned long ulSize;
    std::deque<sScanBuf> sSCanBufDeque;

    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_ScanManBufMutex);
        while(m_dScanManBufDeque.empty())
        {
            m_ScanBufCondition.wait(lock);
        }

        sSCanBufDeque.assign(m_dScanManBufDeque.begin(), m_dScanManBufDeque.end());
        m_dScanManBufDeque.clear();
        lock.unlock();

        if(!m_bMotorRunning)
        {
            sSCanBufDeque.clear();
            continue;
        }

        ulSize = sSCanBufDeque.size();
        double xTemp = 0;
        sPositionData positionData = {0};
        int nDataLen = 0;
        for (int i = 0; i < ulSize; i++)
        {
            sScanBuf ScanBufTemp = sSCanBufDeque[i];

            if(i == 0 || (abs(ScanBufTemp.x - xTemp) >= 0.0001))
            {
                xTemp = ScanBufTemp.x;
                if (m_nControlMode == TASK && xTemp <= m_dTargetDis && xTemp >= 0)
                {
                    nDataLen = TransPosBufData(positionData, ScanBufTemp);
                    OutputPosData(positionData, nDataLen);
                }
//                else if (m_nControlMode == ALARM_LOCATION)
//                {
//                    nDataLen = TransPosBufData(positionData, ScanBufTemp);
//                    AlarmRelocation(positionData, nDataLen);
//                }
            }
        }
        sSCanBufDeque.clear();
    }
}

/*************************************************
Function: CTrackRobot::ManagePosBufData
Description:将激光原始数据转换成对应的侵限值y以及高度z
Input: sPositionData &desPosData 存放处理后的数据， x y[] z[]
       sScanBuf srcBufData      激光原始buf数据
Output: int, y z 数据有效值的数据长度
Others: void
**************************************************/
int CTrackRobot::TransPosBufData(sPositionData &desPosData, const sScanBuf &srcBufData)
{
    double d_y, d_z;
    int nValidSize = 0;

    //里程x值，侵限点定位情况为绝对定位，所以需要单独计算x值。
    if(m_nControlMode == TASK)
    {
        desPosData.x =  srcBufData.x;
        if(m_nDirection == BACKWARD || m_nDirection == BACKWARD_RETURN)
        {
            desPosData.x = m_dTargetDis - desPosData.x;
        }
    }
    else if(m_nControlMode == JOY || m_nControlMode == ALARM_LOCATION)
    {
        double dDistance = 0.0, dMotorOneDis = 0.0;

        int nMotor_One_Displacement = 0;
        int nMotor_Two_Displacement = 0;

        if(m_nMode == PV)
        {
            nMotor_One_Displacement = m_CurrentMotorPos.nMotorOnePos-m_RelocationMotorPos.nMotorOnePos;
            nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_RelocationMotorPos.nMotorTwoPos;
            dMotorOneDis = (nMotor_One_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
            dDistance = ((nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000 +dMotorOneDis)/2;
        }
        else if(m_nMode == PP)
        {
            nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_RelocationMotorPos.nMotorTwoPos;
            dDistance = (nMotor_Two_Displacement/(PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;
        }


        if(!m_bForwardRelocation)
        {
            dDistance = m_dTargetDis + dDistance;
        }
        desPosData.x = dDistance;
    }

    double dInitialAngle = m_dInitialAngle, dPitchAngle = -m_dPitchAngle;
    double d_dy = HALF_OF_TRACK_DISTANCE - LASER_TRACK_POINT_DISTANCE*cos((LASER_TRACK_POINT_ANGLE - dPitchAngle)*M_PI/180);
    double d_dz = LASER_TRACK_POINT_DISTANCE*sin((LASER_TRACK_POINT_ANGLE - dPitchAngle)*M_PI/180);
//    ROS_DEBUG("[dInitialAngle]ddy=%f,ddz=%f",d_dy, d_dz);

//    ROS_INFO("[dInitialAngle]dInitialAngle=%f",dInitialAngle);
    for(int i=0; i< srcBufData.ullRangeSize; i++)
    {
        if(srcBufData.fRanges[i] < 0.1)
            continue;
        d_y = srcBufData.fRanges[i]*sin((dInitialAngle-LASER_ANGLE_INCREMENT*i)*M_PI/180);
        d_y += d_dy;
        d_z = srcBufData.fRanges[i]*cos((dInitialAngle-LASER_ANGLE_INCREMENT*i)*M_PI/180);
        d_z += d_dz;

        if(d_z < 0.35 || d_y < LASER_TRACK_CENTER_DISTANCE + 0.5)
            continue;

        desPosData.y[nValidSize] = (float)d_y;
        desPosData.z[nValidSize] = (float)d_z;
        nValidSize++;
    }

    return nValidSize;
}

/*************************************************
Function: CTrackRobot::OutputPosData
Description: 发布位置数据:x, y[], z[], 并保存至文件大当中
Input: sPositionData &desPosData , 位置数据结构体
       int nDataLen, y z数组对应的有效数据的长度
       int nWriteDataToFile , 是否将位置数据写入到文件当中
Output: void
Others: void
**************************************************/
void CTrackRobot::OutputPosData(sPositionData &desPosData, int nDataLen)
{
    custom_msgs::TrainRobotPosition pos;

    pos.y.resize((unsigned long)nDataLen);
    pos.z.resize((unsigned long)nDataLen);

    pos.x = desPosData.x;

    for(int i = 0; i < nDataLen; i++)
    {
        pos.y[i] = desPosData.y[i];
        pos.z[i] = desPosData.z[i];
    }
    pos.trans_id = 0;

    pos.header.stamp = ros::Time::now();
    pos.header.frame_id = "position";
    m_PositionPublisher.publish(pos);
}

/*************************************************
Function: CTrackRobot::AlarmRelocation
Description: 检测到侵限点后快速停止。
Input: sPositionData &desPosData , 位置数据结构体
       int nDataLen, y z数组对应的有效数据的长度
Output: void
Others: void
**************************************************/
void CTrackRobot::AlarmRelocation(sPositionData &desPosData, int nDataLen)
{
    if (m_nControlMode == ALARM_LOCATION && m_bCheckAlarmData)
    {
        for(int i = 0; i < nDataLen; i++)
        {
            if (desPosData.y[i] < m_dLimitDis)
            {
                QuickStop();
                PublishStatus(RELOCATION_DONE);
                m_nAutoRunStatus = LOCATION_DONE;
                m_bCheckAlarmData = false;

                return;
            }
        }
    }
}

/*************************************************
Function: CTrackRobot::UpdateOriginPosition
Description:更新机器人原点位置值
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::UpdateOriginPosition()
{
    //获取run指令下达时的位置值,并将其置为里程原点值
    SendGetMotorInfoCmd();
    this_thread::sleep_for(std::chrono::milliseconds(20));

    m_MotorPos.nMotorOnePos = m_CurrentMotorPos.nMotorOnePos;
    m_MotorPos.nMotorTwoPos = m_CurrentMotorPos.nMotorTwoPos;

    if(m_nControlMode == TASK)
    {
        m_RelocationMotorPos.nMotorOnePos = m_CurrentMotorPos.nMotorOnePos;
        m_RelocationMotorPos.nMotorTwoPos = m_CurrentMotorPos.nMotorTwoPos;
    }
    m_dDistance = 0.0;
    m_bIsUpdateOrign = true;
    ROS_DEBUG("[UpdateOriginPosition] m_MotorPos.nMotorOnePos=%d,m_MotorPos.nMotorTwoPos=%d",m_MotorPos.nMotorOnePos,m_MotorPos.nMotorTwoPos);
}

/*************************************************
Function: CTrackRobot::CheckStatusThreadFunc
Description:检测手柄消息是否发送结束，检测伺服是否上报报文，伺服重启后给伺服初始化
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::CheckStatusThreadFunc()
{
    ROS_INFO("[CheckStatusThreadFunc] start");
    while(ros::ok())
    {
        timespec CurrentTime;
        long pollInterval;

        //虚拟手柄控制超时检查
        if(m_nControlMode == JOY)
        {
            std::unique_lock<std::mutex> lock(m_JoyTimeOutCheckMutex);
            if(m_tTimeOfLastJoyCmd.tv_sec != 0 && m_tTimeOfLastJoyCmd.tv_nsec != 0)
            {
                timespec_get(&CurrentTime, TIME_UTC);
                pollInterval=((CurrentTime.tv_sec - m_tTimeOfLastJoyCmd.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - m_tTimeOfLastJoyCmd.tv_nsec)) / 1000000;
                if(pollInterval > m_nJoyCtrlTimeoutMSec)
                {
                    m_tTimeOfLastJoyCmd.tv_sec = 0;
                    m_tTimeOfLastJoyCmd.tv_nsec = 0;
                    ROS_DEBUG("[CheckStatusThreadFunc] joy time out, pollInterval=%ld",pollInterval);

                    if (abs(m_dTargetSpeed) <= 0.2)
                    {
                        QuickStop();
                    }
                    else
                    {
                        StopMotion(true);
                    }
                }
            }
            lock.unlock();
        }

        //伺服报文反馈超时监测
        if(m_bIsMotorInit)
        {
            if(m_tTimeOfLastRecvCan.tv_sec != 0 && m_tTimeOfLastRecvCan.tv_nsec != 0)
            {
                timespec_get(&CurrentTime, TIME_UTC);
                pollInterval=((CurrentTime.tv_sec - m_tTimeOfLastRecvCan.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - m_tTimeOfLastRecvCan.tv_nsec)) / 1000000;
                if(pollInterval > 2000)
                {
                    m_tTimeOfLastRecvCan.tv_sec = 0;
                    m_tTimeOfLastRecvCan.tv_nsec = 0;
                    m_bMotorRunning = false;
                    m_bIsMotorInit = false;

                    std::unique_lock<std::mutex> LockSendPack(m_CanTXMutex);
                    if (!m_dCanTXDeque.empty())
                    {
                        m_dCanTXDeque.clear();
                    }
                    LockSendPack.unlock();
                    UsbCan.Reset();
                    ROS_INFO("[CheckStatusThreadFunc] can data receive time out,pollInterval=%ld",pollInterval);
                    m_sMotorOneStatus = "TimeOutErr";
                    m_sMotorTwoStatus = "TimeOutErr";
                }
            }
        }

        bool bEStopStatus = m_bIsEStopButtonPressed;
        //监测急停是否按下
        m_bIsEStopButtonPressed = (GpioControl(GET_EMERGENCY_STOP_BUTTON_STATUS) == 0);
        //急停由按下切换至放开时进行电机伺服初始化
        if(m_bIsEStopButtonPressed != bEStopStatus && bEStopStatus)
        {
            ROS_DEBUG("[CheckStatusThreadFunc] E-Stop is up, call MotorServoInit() ");
            MotorServoInit();
        }

        this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/*************************************************
Function: CTrackRobot::CheckStatusThreadFunc
Description:检测手柄消息是否发送结束，检测伺服是否上报报文，伺服重启后给伺服初始化
Input: void
Output: void
Others: void
**************************************************/
void CTrackRobot::HeartBeatThreadFunc()
{
    ROS_INFO("[HeartBeatThreadFunc] start");
    custom_msgs::TrainRobotHeartBeat heartBeatMsg;
    heartBeatMsg.sender = "track_robot";
    heartBeatMsg.temperature.resize(3);

    while(ros::ok())
    {
        SendCmdOfGetBatteryInfo();

        if(m_bIsMotorInit)
        {
            SendCmdOfGetMotorErrorCode();
            SendHeartBeatCanFrame();
        }
        this_thread::sleep_for(std::chrono::milliseconds(20));

        heartBeatMsg.status = 0x00;
        heartBeatMsg.status |= m_bIsMotorInit ? 0x00:0x01;
        heartBeatMsg.status |= m_bIsEStopButtonPressed ? 0x02:0x00;
        heartBeatMsg.status |= ((m_nBatteryStatus & 0x9F) != 0) ? 0x04:0x00;
        heartBeatMsg.status |= (m_sMotorOneStatus != "normal" || m_sMotorTwoStatus != "normal") ? 0x08:0x00;
        heartBeatMsg.status |= (m_nBatteryCharge == 1) ? 0x10:0x00;

        heartBeatMsg.mode = m_nControlMode;
        heartBeatMsg.velocity_x = m_dCurrentSpeed;
        heartBeatMsg.direction = m_nDirection;

        if(m_nControlMode == TASK)
        {
            heartBeatMsg.position_x =  m_dDistance;
        }
        else if(m_nControlMode == JOY || m_nControlMode == ALARM_LOCATION)
        {
            double dDistance;

            int nMotor_One_Displacement = m_CurrentMotorPos.nMotorOnePos-m_RelocationMotorPos.nMotorOnePos;
            int nMotor_Two_Displacement = m_CurrentMotorPos.nMotorTwoPos-m_RelocationMotorPos.nMotorTwoPos;

            dDistance = ((nMotor_One_Displacement+nMotor_Two_Displacement)/(2.0*PULSES_OF_MOTOR_ONE_TURN*MOTOR_TRANSMISSION))*M_PI*actual_tire_diameter/1000;

            if(!m_bForwardRelocation)
            {
                dDistance = -dDistance;
            }
            heartBeatMsg.position_x = dDistance;
        }

        if(m_nDirection == BACKWARD || m_nDirection == BACKWARD_RETURN)
        {
            heartBeatMsg.position_x = m_dTargetDis - m_dDistance;
        }

        heartBeatMsg.light_status = m_nLightStatus;
        heartBeatMsg.battery_voltage = (m_nBatteryVoltage/10.0);
        heartBeatMsg.battery_quantity = m_nBatteryPower;
        heartBeatMsg.battery_status = m_nBatteryStatus;

        heartBeatMsg.motor_status = "normal";
        if(m_sMotorOneStatus != "normal")
            heartBeatMsg.motor_status = m_sMotorOneStatus;
        if(m_sMotorTwoStatus != "normal")
            heartBeatMsg.motor_status += m_sMotorTwoStatus;

        heartBeatMsg.temperature[0] = m_nBatteryTemp;
        heartBeatMsg.temperature[1] = m_nMotorOneTemp;
        heartBeatMsg.temperature[2] = m_nMotorTwoTemp;

        heartBeatMsg.header.stamp = ros::Time::now();

        m_HeartBeatPublisher.publish(heartBeatMsg);
        this_thread::sleep_for(std::chrono::milliseconds(980));
    }
}

/*************************************************
Function: CTrackRobot::RestartMotorServo
Description:重启伺服驱动器
Input: void
Output: int
Others: void
**************************************************/
int CTrackRobot::RestartMotorServo()
{
    if(GpioControl(POWER_OFF) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn off power electric failed");
        return -1;
    }
    this_thread::sleep_for(std::chrono::milliseconds(500));
    if(GpioControl(LOGIC_OFF) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn off logic electric failed");
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(1000));

    if(GpioControl(LOGIC_ON) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn on logic electric failed");
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(500));
    if(GpioControl(POWER_ON) == -1)
    {
        ROS_ERROR("[RestartMotorServo] turn on power electric failed");
        return -1;
    }

    m_sMotorOneStatus = "normal";
    m_sMotorTwoStatus = "normal";
    return 1;
}

/*************************************************
Function: CTrackRobot::RestartMotorServo
Description:重启伺服驱动器
Input: void
Output: void
Others:
	GPIO名称	    GPIO功能定义            逻辑
输入DI:
	GPIO1_A0	急停按钮信号输入	急停复位-1；急停按下-0
	GPIO1_D0	备用输入（24V）
输出DO:
	GPIO1_A1	驱动器逻辑供电     1-逻辑供电；0-逻辑断电
	GPIO1_A3	灯带B输出	        1-蓝色亮；0-蓝色熄灭
	GPIO1_A4	灯带G输出	        1-绿色亮；0-绿色熄灭
	GPIO1_C6	灯带R输出	        1-红色亮；0-红色熄灭
	GPIO1_C7	驱动器动力供电	    1-动力供电；0-动力断电
	GPIO1_C2	备用输出（24V）
**************************************************/
int CTrackRobot::GpioControl(int nCtrlWord)
{
    CGpioControl GpioCtrl;
    string sDirection = "out";
    int nGpioNum = 0;
    int nGpioValue = 0;

    if(nCtrlWord == GET_EMERGENCY_STOP_BUTTON_STATUS)
    {
        sDirection = "in";
        nGpioNum = GPIO1_A0;
        GpioCtrl.SetPinAndDirection(nGpioNum, sDirection);

        return GpioCtrl.GetGpioValue();
    }

    switch(nCtrlWord)
    {
        case LOGIC_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_A1;
            break;
        case LOGIC_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_A1;
            break;
        case R_LIGHT_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_C6; m_nLightStatus = 0;
            break;
        case R_LIGHT_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_C6;
            break;
        case G_LIGHT_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_A4; m_nLightStatus = 1;
            break;
        case G_LIGHT_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_A4;
            break;
        case B_LIGHT_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_A3; m_nLightStatus = 2;
            break;
        case B_LIGHT_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_A3;
            break;
        case POWER_ON:
            nGpioValue = HIGH; nGpioNum = GPIO1_C7;
            break;
        case POWER_OFF:
            nGpioValue = LOW; nGpioNum = GPIO1_C7;
            break;
        default:
            ROS_WARN("[GpioControl] ctrl word error");
            return -1;
    }

    timespec StartTime;
    timespec_get(&StartTime, TIME_UTC);

    GpioCtrl.SetPinAndDirection(nGpioNum, sDirection);
    while(GpioCtrl.GetGpioValue() != nGpioValue)
    {
        GpioCtrl.SetGpioValue(HIGH);
        this_thread::sleep_for(std::chrono::milliseconds(10));

        timespec CurrentTime;
        timespec_get(&CurrentTime, TIME_UTC);

        long pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec) * 1000000000 \
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000;
        if(pollInterval > 100)
        {
            ROS_ERROR("[GpioControl] set value timeout error");
            return -1;
        }
    }
    return 1;
}

CTrackRobot::~CTrackRobot()
{
    ROS_DEBUG("[CTrackRobot][~CTrackRobot] begin...");
    UsbCan.Close();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    m_pCANReceiveThread->join();
#if USE_CAN2_RT
    m_pCAN2ReceiveThread->join();
#endif
    m_pCANManageThread->join();
    m_pCANSendThread->join();
    m_pControlThread->join();
    m_pAutoRunThread->join();
    m_pPositionFilterThread->join();
    m_pCheckStatusThread->join();
    m_pHeartBeatThread->join();
    m_pScanThread->join();

    delete m_pCANReceiveThread;
#if USE_CAN2_RT
    delete m_pCAN2ReceiveThread;
#endif
    delete m_pCANManageThread;
    delete m_pCANSendThread;
    delete m_pControlThread;
    delete m_pAutoRunThread;
    delete m_pPositionFilterThread;
    delete m_pCheckStatusThread;
    delete m_pHeartBeatThread;
    delete m_pScanThread;
}

/*************************************************
Function: CTrackRobot::updata_XML
Description:修改配置文件中的轮子直径
Input:
    path 配置文件名
    updataposion，从外到更新节点的路径
    attributes,更新元素

Output: 执行结果
**************************************************/
int CTrackRobot::updata_XML(string paramName, string paramValue)
{
    XMLDocument doc;
    char buf[100];
    getcwd(buf,100);
    strcat(buf,"/../catkin_ws/src/track_robot/launch/track_robot.launch");
    const char* path_updata = buf;
    const char* paranName_updata = paramName.c_str();
    const char* paramValue_updata = paramValue.c_str();
    if (doc.LoadFile(path_updata))
    {
        ROS_DEBUG("the launch file is:%s",buf);
        return 0;
    }
    XMLElement *current_root = doc.RootElement();
    XMLElement *nodeName = current_root->FirstChildElement("node");
    if(NULL==nodeName)
        return 0;
    XMLElement *paramNode = nodeName->FirstChildElement("param");
    while (paramNode!=NULL)
    {
        string str = paramNode->Attribute("name");
        if(str.compare(paranName_updata)==0)
        {
            paramNode->SetAttribute("value",paramValue_updata);
            break;
        }
        paramNode = paramNode->NextSiblingElement();
    }

    doc.SaveFile(path_updata, false);
    return 1;
}

