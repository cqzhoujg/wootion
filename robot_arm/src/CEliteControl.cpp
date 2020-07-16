//
// Created by zhoujg on 19-12-6.
//

#include "CEliteControl.h"

extern bool bExit;

CEliteControl::CEliteControl():
    m_bRecordDragTrack(false),
    m_bBusy(false),
    m_bIgnoreMove(false),
    m_bIsRecordReset(false),
    m_bEmeStop(false),
    m_bArmInit(false),
    m_nDragStatus(DISABLE),
    m_nEliteState(ALARM),
    m_nEliteMode(Remote),
    m_nTaskName(IDLE),
    m_sOrbitFileName("arm_orbit.txt"),
    m_sResetFile("arm_reset.txt"),
    m_sResetOrbitFile("null"),
    m_sStatus("idle")
{
    ros::NodeHandle PublicNodeHandle;
    ros::NodeHandle PrivateNodeHandle("~");

    PrivateNodeHandle.param("node_name", m_sNodeName, ros::this_node::getName());
    PrivateNodeHandle.param("check_receiver", m_nCheckReceiver, 0);
    PrivateNodeHandle.param("elite_ip", m_sEliteRobotIP, std::string("192.168.100.201"));
    PrivateNodeHandle.param("elite_port", m_nElitePort, 8055);
    PrivateNodeHandle.param("timeout_len", m_nTimeoutLen, 60);
    PrivateNodeHandle.param("p_level", m_nSmoothnessLevel, 7);
    PrivateNodeHandle.param("elt_speed", m_dEltSpeed, 25.0);
    PrivateNodeHandle.param("rotate_speed", m_dRotateSpeed, 5.0);
    PrivateNodeHandle.param("rotate_limit_angle", m_dRotateLimitAngle, 30.0);
    PrivateNodeHandle.param("print_level", m_sPrintLevel, std::string("debug"));
    PrivateNodeHandle.param("track_path", m_sArmTrackPath, std::string(getenv("HOME")).append("/track/"));
    PrivateNodeHandle.param("orbit_step", m_dOrbitStep, 3.0);
    PrivateNodeHandle.param("arm_origin", m_sArmOrigin, std::string("0,-150,130,-160,90,0"));
    PrivateNodeHandle.param("debug_teach", m_nDebugTeach, 0);
    PrivateNodeHandle.param("filter_value", m_dFilterValue, 20.0);

    PublicNodeHandle.param("arm_service", m_sArmService, std::string("arm_control"));
    PublicNodeHandle.param("arm_cmd", m_sArmCmdTopic, std::string("arm_cmd"));
    PublicNodeHandle.param("arm_ack", m_sArmAckTopic, std::string("arm_ack"));
    PublicNodeHandle.param("arm_heart", m_sHeartBeatTopic, std::string("arm_heart_beat"));
    PublicNodeHandle.param("arm_abnormal", m_sAbnormalTopic, std::string("arm_exception"));
    PublicNodeHandle.param("agv_status", m_sAgvStatusTopic, std::string("robot_status"));

    ROS_INFO("[ros param] node_name:%s", m_sNodeName.c_str());
    ROS_INFO("[ros param] check_receiver:%d", m_nCheckReceiver);
    ROS_INFO("[ros param] elite_ip:%s", m_sEliteRobotIP.c_str());
    ROS_INFO("[ros param] elite_port:%d",m_nElitePort);
    ROS_INFO("[ros param] timeout_len:%d",m_nTimeoutLen);
    ROS_INFO("[ros param] p_level:%d",m_nSmoothnessLevel);
    ROS_INFO("[ros param] elt_speed:%f",m_dEltSpeed);
    ROS_INFO("[ros param] rotate_speed:%f",m_dRotateSpeed);
    ROS_INFO("[ros param] rotate_limit_angle:%f",m_dRotateLimitAngle);
    ROS_INFO("[ros param] print_level:%s", m_sPrintLevel.c_str());
    ROS_INFO("[ros param] track_path:%s", m_sArmTrackPath.c_str());
    ROS_INFO("[ros param] orbit_step:%f", m_dOrbitStep);
    ROS_INFO("[ros param] arm_origin:%s", m_sArmOrigin.c_str());
    ROS_INFO("[ros param] debug_teach:%d", m_nDebugTeach);
    ROS_INFO("[ros param] filter_value:%f", m_dFilterValue);

    ROS_INFO("[ros param] arm_service:%s", m_sArmService.c_str());
    ROS_INFO("[ros param] arm_cmd:%s", m_sArmCmdTopic.c_str());
    ROS_INFO("[ros param] arm_ack:%s", m_sArmAckTopic.c_str());
    ROS_INFO("[ros param] arm_heart:%s", m_sHeartBeatTopic.c_str());
    ROS_INFO("[ros param] arm_abnormal:%s", m_sAbnormalTopic.c_str());
    ROS_INFO("[ros param] agv_status:%s", m_sAgvStatusTopic.c_str());

    if(-1 == CreateDataPath())
    {
        ROS_ERROR("[CEliteControl] create data path failed");
        exit(-1);
    }

    double dTimeout = 60.0;
    wootion_msgs::RobotStatus::ConstPtr AgvStatus = ros::topic::waitForMessage<wootion_msgs::RobotStatus>(m_sAgvStatusTopic, ros::Duration(dTimeout));

    if(AgvStatus != nullptr)
    {
        m_bEmeStop = ((AgvStatus->warn_status[3] & 0x01) > 0);
    }
    else
    {
        ROS_ERROR("[CEliteControl] wait for msg robot_status timeout: %fs.",dTimeout);
        exit(-1);
    }

    if(!Init())
    {
        ROS_ERROR("[CEliteControl] elite init failed");
        UnInit();
        exit(-1);
    }

    m_tRecordDataTime.tv_sec = 0;
    m_tRecordDataTime.tv_nsec = 0;
    m_sResetFile = m_sArmTrackPath+m_sResetFile;

    m_ArmService = PublicNodeHandle.advertiseService(m_sArmService, &CEliteControl::ArmServiceFunc, this);

    m_ArmCmdSubscriber = PublicNodeHandle.subscribe<wootion_msgs::GeneralCmd>(m_sArmCmdTopic, 10, boost::bind(&CEliteControl::ArmCmdCallBack, this, _1));
    m_AgvStatusSubscriber = PublicNodeHandle.subscribe<wootion_msgs::RobotStatus>(m_sAgvStatusTopic, 1, boost::bind(&CEliteControl::AgvStatusCallBack, this, _1));

    m_ArmAckPublisher = PublicNodeHandle.advertise<wootion_msgs::GeneralAck>(m_sArmAckTopic, 10);
    m_HeartBeatPublisher = PublicNodeHandle.advertise<wootion_msgs::GeneralTopic>(m_sHeartBeatTopic, 10);
    m_AbnormalPublisher = PublicNodeHandle.advertise<wootion_msgs::GeneralTopic>(m_sAbnormalTopic, 10);

    this_thread::sleep_for(std::chrono::milliseconds(500));

    try
    {
        m_pMonitorThread = new std::thread(std::bind(&CEliteControl::MonitorThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CEliteControl] malloc monitor thread failed, %s", exception.what());
        UnInit();
        exit(-1);
    }

    try
    {
        m_pEliteStatusThread = new std::thread(std::bind(&CEliteControl::UpdateEliteThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CEliteControl] malloc update-elite thread failed, %s", exception.what());
        UnInit();
        exit(-1);
    }

    try
    {
        m_pHeartBeatThread = new std::thread(std::bind(&CEliteControl::HeartBeatThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CEliteControl] malloc heart beat thread failed, %s", exception.what());
        UnInit();
        exit(-1);
    }

    this_thread::sleep_for(std::chrono::milliseconds(1000));

    if(!m_bEmeStop)
    {
        string sOutput;
        if(!ResetToOrigin(sOutput))
        {
            ROS_ERROR("[CEliteControl] %s",sOutput.c_str());
            UnInit();
            exit(-1);
        }
        memcpy(m_RotateOriginPos, m_EliteCurrentPos, sizeof(m_RotateOriginPos));
    }
    ROS_INFO("[CEliteControl] done");
}

/*************************************************
Function: CEliteControl::Init
Description: 艾利特机器人初始化
Input: void
Output: bool  true  初始化成功
              false 初始化失败
Others: void
**************************************************/
bool CEliteControl::Init()
{
    ROS_INFO("[Init] start");
    m_bArmInit = false;
    elt_error err;
    int nRet = 0;
    //创建连接
    m_eltCtx = elt_create_ctx(m_sEliteRobotIP.c_str(), m_nElitePort);
    if (nullptr == m_eltCtx)
    {
        ROS_INFO("[Init] elt_create_ctx failed");
    }
    //登录
    nRet = elt_login(m_eltCtx);
    if (ELT_SUCCESS == nRet)
    {
        m_nEliteState = DROP_LINE;
        ROS_INFO("[Init] elt login succeed");
    }
    else
    {
        ROS_ERROR("[Init] elt login failed. ret=%d,err.code=%d,err.msg=%s", nRet, err.code, err.err_msg);
        return false;
    }

    if (UpdateEltOrigin() == -1)
    {
        ROS_ERROR("[Init] UpdateEltOrigin error");
        return false;
    }

    if(m_bEmeStop)
    {

        ROS_WARN("[Init] e-stop button is pressed, only login elite");
        return true;
    }

    //清除报警, 3次重试。
    if(EliteClearAlarm() == -1)
    {
        ROS_ERROR("[Init] clear elite alarm failed");
        return false;
    }

    //同步数据, 3次重试
    if(EliteSyncMotorStatus() == -1)
    {
        ROS_ERROR("[Init] sync elite motor status failed");
        return false;
    }

    //打开伺服, 3次重试
    if(EliteOpenServo() == -1)
    {
        ROS_ERROR("[Init] open elite servo failed");
        return false;
    }

    m_bArmInit = true;
    return true;
}

/*************************************************
Function: CEliteControl::UnInit
Description: 艾利特机器人失能
Input: void
Output: void
Others: void
**************************************************/
void CEliteControl::UnInit()
{
    //退出
    int ret = elt_logout(m_eltCtx);
    if (ELT_SUCCESS == ret)
    {
        ROS_INFO("[UnInit] elt_logout succeed");
    }
    else
    {
        ROS_INFO("elt_logout failed.");
    }

    //销毁链接
    ret = elt_destroy_ctx(m_eltCtx);
    if (ELT_SUCCESS == ret)
    {
        ROS_INFO("[UnInit] elt_destroy_ctx succeed");
    }
    else
    {
        ROS_INFO("[UnInit] elt_destroy_ctx failed.");
    }
}

/*************************************************
Function: CEliteControl::UpdateEliteThreadFunc
Description: 艾利特机器人状态刷新线程函数
Input: void
Output: void
Others: void
**************************************************/
void CEliteControl::UpdateEliteThreadFunc()
{
    ROS_INFO("[UpdateEliteThreadFunc] start");
    int ret;
    elt_error err;
    elt_robot_pos OldRecordPos, LastPos;
    std::unique_lock<std::mutex> Lock(m_ResetFileMutex);
    if(m_ResetFile.OpenFile(m_sResetFile, O_CREAT | O_RDWR) == -1)
    {
        ROS_ERROR("[UpdateEliteThreadFunc] open reset file error:%s", m_sResetFile.c_str());
        UnInit();
        exit(-1);
    }
    Lock.unlock();
    m_bWriteOrigin = true;

    memset(OldRecordPos, 0, sizeof(OldRecordPos));
    memset(&LastPos, 0, sizeof(LastPos));
    memset(&m_EliteCurrentPos, 0, sizeof(m_EliteCurrentPos));
    int nErrorTimes = 0;
//    int nEltMode = Remote;

    while(ros::ok())
    {
        if(nErrorTimes >= 3)
        {
            ROS_ERROR("[UpdateEliteThreadFunc] process exit, get elite info failed, times=%d",nErrorTimes);
            UnInit();
            exit(-1);
        }
        //获取elt工作状态
        ret = elt_get_robot_state(m_eltCtx, &m_nEliteState, &err);
        if (ELT_SUCCESS != ret)
        {
            ROS_WARN("[UpdateEliteThreadFunc] get elite state failed");
            nErrorTimes++;
            this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        //获取elt控制模式
//        ret = elt_get_robot_mode(m_eltCtx, &m_nEliteMode, &err);
//        if (ELT_SUCCESS != ret)
//        {
//            ROS_WARN("[UpdateEliteThreadFunc] get elite mode failed");
//            nErrorTimes++;
//            this_thread::sleep_for(std::chrono::milliseconds(200));
//            continue;
//        }
//        ROS_WARN("[UpdateEliteThreadFunc] get elite mode :%d",m_nEliteMode);

        //获取elt各个轴的绝对位置信息
        if(GetElitePos(m_EliteCurrentPos) == -1)
        {
            ROS_WARN("[UpdateEliteThreadFunc] get elite current pos failed");
            nErrorTimes++;
            this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        nErrorTimes = 0;
//        if(nEltMode != Remote && m_nEliteMode == Remote)
//        {
//            ROS_INFO("[UpdateEliteThreadFunc] elt mode changed ro remote, update the rotate origin");
//            memcpy(m_RotateOriginPos, m_EliteCurrentPos, sizeof(m_RotateOriginPos));
//        }

//        nEltMode = m_nEliteMode;

        string sAxisData;
        bool bIsWrite = false;
        for(int i=0; i<ROBOT_POSE_SIZE; i++)
        {
            //只有当角度差大于0.1时才将位置信息写入轨迹录制文件，剔除重复数据，剔除超限位数据
            if((abs(m_EliteCurrentPos[i] - OldRecordPos[i]) > m_dOrbitStep) &&\
                m_EliteCurrentPos[i] <= AxisLimitAngle[i] && m_EliteCurrentPos[i] >= AxisLimitAngle[i+6])
            {
                bIsWrite = true;
            }

            sAxisData.append(to_string(m_EliteCurrentPos[i]));
            if(i != ROBOT_POSE_SIZE - 1)
            {
                sAxisData.append(" ");
            }
        }
        sAxisData.append("\n");

        if(CheckOrigin(LastPos) && !CheckOrigin(m_EliteCurrentPos))
        {
            m_bIsRecordReset = true;
        }

        if(m_bRecordDragTrack)
        {
            //轨迹记录超时检测
            if(m_tRecordDataTime.tv_sec != 0 && m_tRecordDataTime.tv_nsec != 0)
            {
                timespec CurrentTime;
                timespec_get(&CurrentTime, TIME_UTC);

                __time_t pollInterval = (CurrentTime.tv_sec - m_tRecordDataTime.tv_sec) \
                                    +(CurrentTime.tv_nsec - m_tRecordDataTime.tv_nsec) / 1000000000;
                if(pollInterval > m_nTimeoutLen*3)
                {
                    m_bRecordDragTrack = false;
                    m_tRecordDataTime.tv_sec = 0;
                    m_tRecordDataTime.tv_nsec = 0;
                    ROS_INFO("[UpdateEliteThreadFunc] record track time out,pollInterval=%ld s",pollInterval);

                    if(EliteDrag(DISABLE) == -1)
                    {
                        ROS_ERROR("[UpdateEliteThreadFunc] disable drag failed");
                    }
                    if(m_TrackFile.CloseFile() == -1)
                    {
                        ROS_ERROR("[UpdateEliteThreadFunc] close file failed");
                    }
                }
            }
            if(bIsWrite)
            {
                m_TrackFile.Output(sAxisData);
                memcpy(OldRecordPos, m_EliteCurrentPos, sizeof(OldRecordPos));
            }
        }

        if(m_bIsRecordReset && m_nTaskName != RESET && m_nTaskName != ROTATE && bIsWrite)
        {
            if(m_bWriteOrigin)
            {
                string sOriginData;
                for(int i=0; i<ROBOT_POSE_SIZE; i++)
                {
                    sOriginData.append(to_string(m_EltOriginPos[i]));
                    if(i != ROBOT_POSE_SIZE - 1)
                    {
                        sOriginData.append(" ");
                    }
                }
                sOriginData.append("\n");
                m_ResetFile.Output(sOriginData);
                m_bWriteOrigin = false;
            }
            m_ResetFile.Output(sAxisData);
            memcpy(OldRecordPos, m_EliteCurrentPos, sizeof(OldRecordPos));
        }

        memcpy(LastPos, m_EliteCurrentPos, sizeof(LastPos));

        this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/*************************************************
Function: CEliteControl::MonitorThreadFunc
Description: 信号量监听线程函数
Input: void
Output: void
Others: void
**************************************************/
void CEliteControl::MonitorThreadFunc()
{
    ROS_INFO("[MonitorThreadFunc] start");
    while(ros::ok())
    {
        if(bExit)
        {
            printf("[MonitorThreadFunc] exit...\n");
            UnInit();
            exit(-1);
        }
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

/*************************************************
Function: CEliteControl::HeartBeatThreadFunc
Description: 机械臂心跳消息,每1s发布一次状态消息
Input: void
Output: void
Others: void
**************************************************/
void CEliteControl::HeartBeatThreadFunc()
{
    ROS_INFO("[HeartBeatThreadFunc] start");
    int nTimes = 0;
    string sPreOrigin = "true";

    while(ros::ok())
    {
        //心跳消息
        wootion_msgs::GeneralTopic HeartBeatMsg;
        boost::property_tree::ptree ptAllItem;
        bool bStatusChanged = false;
        bool bOriginStatusChanged = false;

        HeartBeatMsg.header.stamp = ros::Time::now();
        HeartBeatMsg.sender = "robot_arm";
        HeartBeatMsg.receiver = "";
        HeartBeatMsg.trans_id = 1;
        HeartBeatMsg.data.clear();

        //状态
        string sStatus = vsArmStatus[IDLE];
        if(m_nEliteState == MOVING || m_nTaskName == RECORD || m_bIgnoreMove)
        {
            sStatus = vsArmStatus[m_nTaskName];
        }
        if(m_nEliteState == ALARM || m_nEliteState == PAUSE)
        {
            sStatus = vsArmStatus[ERROR];
        }
        if(m_sStatus != sStatus)
            bStatusChanged = true;

        m_sStatus = sStatus;

        ptAllItem.put("status", m_sStatus);

        string sMode = "remote";
        if(m_nEliteMode != Remote)
            sMode = m_nEliteMode == Teach ? "teach" : "play";
        if(1 == m_nDebugTeach)
        {
            sMode = "teach";
        }
        ptAllItem.put("mode", sMode);

        //错误信息
        string sErrCode, sErrMsg;
        if(m_nEliteState == ALARM)
        {
            sErrCode = "1";
            sErrMsg = "alarm";
        }
        else if(m_nEliteState == PAUSE)
        {
            sErrCode = "2";
            sErrMsg = "emergency_stop";
        }
        ptAllItem.put("error_code", sErrCode);
        ptAllItem.put("error_msg", sErrMsg);

        //是否在原点
        string sOrigin;
        if(CheckOrigin(m_EliteCurrentPos))
        {
            sOrigin = "true";
        }
        else
        {
            sOrigin = "false";
        }
        if(sPreOrigin != sOrigin)
            bOriginStatusChanged = true;

        ptAllItem.put("origin", sOrigin);
        sPreOrigin = sOrigin;

        //是否在原点,只判断前3个轴,用于前台检测是否有轨迹
        string sBaseOrigin;
        bool bCheckBaseOrigin = true;
        if(CheckOrigin(m_EliteCurrentPos, bCheckBaseOrigin))
        {
            sBaseOrigin = "true";
        }
        else
        {
            sBaseOrigin = "false";
        }
        ptAllItem.put("base_origin", sBaseOrigin);

        //是否有轨迹
        string sOrbitFileExist;
        if(access((m_sArmTrackPath + m_sOrbitFileName).c_str(), 0) == 0 || access(m_sResetOrbitFile.c_str(), 0) == 0)
        {
            sOrbitFileExist = "true";
        }
        else
        {
            sOrbitFileExist = "false";
        }

        ptAllItem.put("exist_orbit", sOrbitFileExist);

        //当前位置点
        string sCurrentData;
        for(int i=0; i<6; i++)
        {
            sCurrentData.append(to_string(m_EliteCurrentPos[i]));
            if(i != 5)
            {
                sCurrentData.append(",");
            }
        }
        ptAllItem.put("current_pos", sCurrentData);

        std::stringstream ssStream;
        boost::property_tree::write_json(ssStream, ptAllItem);
        HeartBeatMsg.data = ssStream.str();

        if(20 == nTimes || bStatusChanged || bOriginStatusChanged)
        {
            m_HeartBeatPublisher.publish(HeartBeatMsg);
            nTimes = 0;
        }

        //异常检测
        if(m_bRecordDragTrack && m_nEliteState == ALARM)
        {
            wootion_msgs::GeneralTopic AbnormalMsg;
            AbnormalMsg.header.stamp = ros::Time::now();
            AbnormalMsg.sender = "robot_arm";
            AbnormalMsg.receiver = "";
            AbnormalMsg.trans_id = 2;
            AbnormalMsg.data.clear();

            AbnormalMsg.data.append("{record_status:alarm}");

            m_AbnormalPublisher.publish(AbnormalMsg);
        }

        nTimes++;
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/*************************************************
Function: CEliteControl::UpdateEltOrigin
Description: 从文件处更新机械臂原点
Input: void
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::UpdateEltOrigin()
{
    ROS_INFO("[UpdateEltOrigin] start");
    vector<string> vCmdList;
    boost::split(vCmdList, m_sArmOrigin, boost::is_any_of(","), boost::token_compress_on);

    if(vCmdList.size() < 6)
    {
        ROS_ERROR("[EliteRunDragTrack] arm origin error:%s", m_sArmOrigin.c_str());
        return -1;
    }
    else
    {
        for(int i=0; i<ROBOT_POSE_SIZE; i++)
        {
            m_EltOriginPos[i] = stod(vCmdList[i]);
        }
    }
    PrintJointData(m_EltOriginPos, "UpdateEltOrigin");
    ROS_INFO("[UpdateEltOrigin] end");
}

/*************************************************
Function: CEliteControl::EliteJointMove
Description: 艾利特机器人节点运动方式封装
Input: elt_robot_pos &targetPos 目标距离
       double dSpeed 移动速度
       string &sErr 错误返回信息
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteJointMove(elt_robot_pos &targetPos, double dSpeed, string &sErr)
{
    ROS_INFO("[EliteMultiPointMove] start.");
    int ret;
    elt_error err;

    //6轴运动限位检测，防止报警
    for(int i=0; i<6 ;i++)
    {
        if(targetPos[i] > AxisLimitAngle[i] || targetPos[i] < AxisLimitAngle[i+6])
        {
            sErr.append("target pos error,the ").append(to_string(i+1)).append("th axis beyond the limit");
            ROS_ERROR("[EliteJointMove] %s", sErr.c_str());
            return -1;
        }
    }

    ret = elt_joint_move( m_eltCtx, targetPos, dSpeed, &err);

    if (ret != ELT_SUCCESS)
    {
        if(err.err_msg[0] != '\0')
            sErr.append(err.err_msg);
        else
            sErr.append("elt joint move error");

        ROS_ERROR("[EliteJointMove] err code: %d,err message: %s", err.code, err.err_msg);
        return -1;
    }

    this_thread::sleep_for(std::chrono::milliseconds(100));

    ROS_INFO("[EliteJointMove] after call joint move");
    return 1;
}

/*************************************************
Function: CEliteControl::EliteMultiPointMove
Description: 根据目标位置及当前位置插补出平滑轨迹点,并调用艾利特的轨迹运动函数
Input: elt_robot_pos &targetPos 目标距离
       double dSpeed 移动速度
       string &sErr 错误返回信息
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteMultiPointMove(elt_robot_pos &targetPos, double dSpeed, string &sErrMsg)
{
    ROS_INFO("[EliteMultiPointMove] start");
    elt_error err;
    int ret;
    double dStepValue = m_dOrbitStep;

    //6轴运动限位检测，防止报警
    for(int i=0; i<6 ;i++)
    {
        if(targetPos[i] > AxisLimitAngle[i] || targetPos[i] < AxisLimitAngle[i+6])
        {
            sErrMsg.append("target pos error,the ").append(to_string(i+1)).append("th axis beyond the limit");
            ROS_ERROR("[EliteMultiPointMove] %s", sErrMsg.c_str());
            return -1;
        }
    }

    //设置轨迹路点运动的速度
    ret = elt_set_waypoint_max_joint_speed( m_eltCtx, dSpeed, &err);

    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "set speed failed";
        ROS_ERROR("[EliteMultiPointMove] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }

    //清除轨迹点集
    ret = elt_clear_waypoint( m_eltCtx, &err);
    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "clear way point failed";
        ROS_ERROR("[EliteMultiPointMove] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }

    elt_robot_pos currentPos, targetPosTemp;
    memcpy(currentPos, m_EliteCurrentPos, sizeof(currentPos));
    memcpy(targetPosTemp, m_EliteCurrentPos, sizeof(currentPos));
//    PrintJointData(currentPos, "currentPos");
//    PrintJointData(targetPos, "targetPos");

    //对目标只做线性差值
    for(int i=0; i<6; i++)
    {
        while(ros::ok())
        {
            if(targetPos[i] - currentPos[i] >= 0)
            {
                targetPosTemp[i] += dStepValue;
                if(targetPosTemp[i] - targetPos[i] > 0.001)
                {
                    targetPosTemp[i] = targetPos[i];
                }
            }
            else
            {
                targetPosTemp[i] -= dStepValue;
                if(targetPosTemp[i] - targetPos[i] < 0.001)
                {
                    targetPosTemp[i] = targetPos[i];
                }
            }

            ret = elt_add_waypoint( m_eltCtx, targetPosTemp, &err);
//            PrintJointData(targetPosTemp, "EliteMultiPointMove");

            if (ret != ELT_SUCCESS)
            {
                sErrMsg.append("elt add way point error");
                ROS_ERROR("[EliteMultiPointMove] add way point err code: %d,err message: %s", err.code, err.err_msg);
                return -1;
            }
            if(abs(targetPosTemp[i] - targetPos[i]) <= 0.0001)
            {
                break;
            }
        }
    }

    //执行轨迹
    int nMoveType = 0;
    ret = elt_track_move( m_eltCtx, nMoveType, m_nSmoothnessLevel, &err);

    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "elt track move error";
        ROS_INFO("[EliteMultiPointMove] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }
    ROS_INFO("[EliteMultiPointMove] call elt track move");

    this_thread::sleep_for(std::chrono::milliseconds(100));
    return 1;
}

/*************************************************
Function: CEliteControl::GetElitePos
Description: 艾利特机器人各个关节角度信息获取
Input: elt_robot_pos &pos_array, 保存角度位置信息
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::GetElitePos(elt_robot_pos &pos_array)
{
    int ret;
    elt_error err;

    //获取机器人当前关节角度
    ret = elt_get_robot_pos(m_eltCtx, pos_array, &err);
    if (ELT_SUCCESS != ret)
    {
        ROS_INFO("[GetElitePos] get robot pos failed, err code: %d,err message: %s", err.code, err.err_msg);
        return -1;
    }
    return 1;
}

/*************************************************
Function: CEliteControl::EliteStop
Description: 艾利特机器人停止运动二次封装
Input: string sErr,失败时的反馈信息
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteStop(string &sErr)
{
    int ret;
    elt_error err;

    if(m_nEliteState == PAUSE)
    {
        //清除轨迹点集
        ret = elt_clear_waypoint( m_eltCtx, &err);
        if (ret != ELT_SUCCESS)
        {
            sErr = "clear way point failed";
            ROS_ERROR("[EliteStop] %s,err code: %d,err message: %s",sErr.c_str(), err.code, err.err_msg);
            return -1;
        }
        ret = elt_run(m_eltCtx, &err);
        if (ELT_SUCCESS != ret)
        {
            if(err.err_msg[0] != '\0')
                sErr.append(err.err_msg);
            else
                sErr.append("elt run error");

            ROS_INFO("[EliteStop] elt run failed, err code: %d,err message: %s", err.code, err.err_msg);
            return -1;
        }
    }

    ret = elt_stop(m_eltCtx, &err);
    if (ELT_SUCCESS != ret)
    {
        if(err.err_msg[0] != '\0')
            sErr.append(err.err_msg);
        else
            sErr.append("elt stop move error");

        ROS_INFO("[EliteStop] elt stop failed, err code: %d,err message: %s", err.code, err.err_msg);
        return -1;
    }
    return 1;
}

/*************************************************
Function: CEliteControl::EliteDrag
Description: 艾利特机器人停止运动二次封装
Input: int nCmd,控制信息,使能 or 失能
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteDrag(int nCmd)
{
    int ret;
    elt_error err;
    if(nCmd == ENABLE && m_nDragStatus == DISABLE)
    {
        ret = elt_set_servo_status(m_eltCtx, DISABLE, &err);
        if (ELT_SUCCESS != ret)
        {
            ROS_ERROR("[EliteDrag] disable elt servo failed. ret=%d, err.code=%d, err.msg=%s", ret, err.code, err.err_msg);
            return -1;
        }

        this_thread::sleep_for(std::chrono::milliseconds(300));

        ret = elt_drag_teach(m_eltCtx, nCmd, &err);
        if (ELT_SUCCESS != ret)
        {
            ROS_INFO("[EliteDrag] elt drag teach failed, err code: %d,err message: %s", err.code, err.err_msg);
            return -1;
        }

        timespec_get(&m_tRecordDataTime, TIME_UTC);
        m_bRecordDragTrack = true;
        m_nDragStatus = ENABLE;
    }
    else if(nCmd == DISABLE && m_nDragStatus == ENABLE)
    {
        m_bRecordDragTrack = false;
        m_tRecordDataTime.tv_sec = 0;
        m_tRecordDataTime.tv_nsec = 0;

        ret = elt_drag_teach(m_eltCtx, nCmd, &err);
        if (ELT_SUCCESS != ret)
        {
            ROS_INFO("[EliteDrag]  elt drag teach failed, err code: %d,err message: %s", err.code, err.err_msg);
            return -1;
        }

        if(EliteSyncMotorStatus() == -1)
        {
            ROS_INFO("[EliteDrag] sync elite motor status failed");
            return -1;
        }

        //打开伺服,3次重试
        if(EliteOpenServo() == -1)
        {
            ROS_ERROR("[EliteDrag] open elite servo failed");
            return -1;
        }

        m_nDragStatus = DISABLE;
    }
    return 1;
}

/*************************************************
Function: CEliteControl::EliteRunDragTrack
Description: 复现拖拽轨迹
Input: string &sFileName 轨迹文件名称
        double dSpeed, 运动速度
        int nDirection, 播放轨迹的方向
        string &sErrMsg, 播放失败的反馈信息
        string sPlayFirstAxis, 是否播放第一轴
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteRunDragTrack(const string &sFileName, double dSpeed, int nDirection, string &sErrMsg, string sPlayFirstAxis)
{
    ROS_INFO("[EliteRunDragTrack] start");
    elt_error err;
    int ret;
    ifstream trackFile;

    //打开轨迹文件
    trackFile.open(sFileName.c_str(), ios::in);
    if (!trackFile.is_open())
    {
        trackFile.close();
        sErrMsg = "cannot open orbit file:";
        sErrMsg.append(sFileName);
        ROS_INFO("[EliteRunDragTrack] %s",sErrMsg.c_str());
        return -1;
    }

    ROS_INFO("[EliteRunDragTrack] opened file %s",sFileName.c_str());

    //设置轨迹路点运动的速度
    ret = elt_set_waypoint_max_joint_speed( m_eltCtx, dSpeed,&err);

    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "set speed failed";
        ROS_INFO("[EliteRunDragTrack] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }

    //清除轨迹点集
    ret = elt_clear_waypoint( m_eltCtx, &err);
    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "clear way point failed";
        ROS_INFO("[EliteRunDragTrack] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }

    //将轨迹文件读取到双向队列当中
    EltPos prePos;
    memset(&prePos, 0, sizeof(prePos));

    deque<EltPos> trackDeque;
    while (!trackFile.eof())
    {
        EltPos targetPos;
        memset(&targetPos, 0, sizeof(targetPos));

        for(int i=0; i< ROBOT_POSE_SIZE; i++)
        {
            trackFile >> targetPos.eltPos[i];
        }

        if(sPlayFirstAxis == "0")
        {
            targetPos.eltPos[0] = m_EliteCurrentPos[0];
        }

        int bValid = false;
        for(int i=0; i<6; i++)
        {
            if(abs(prePos.eltPos[i] - targetPos.eltPos[i]) > 0.1)
            {
                bValid = true;
                break;
            }
        }

        if(bValid)
        {
            trackDeque.push_back(targetPos);
            memcpy(&(prePos.eltPos[0]), &(targetPos.eltPos[0]), sizeof(prePos.eltPos));
        }
    }
    trackFile.close();

    if(trackDeque.size() > 1)//读取时会重复读取最后一行
        trackDeque.pop_back();

    if(trackDeque.size() <= 1)
    {
        sErrMsg = "empty";
        ROS_INFO("[EliteRunDragTrack] reset orbit file is empty");
        return -1;
    }

    //剔除机械臂bug引入的异常错误点
    RemoveErrPoints(trackDeque);

    //根据方向，调用elt库函数，将队列中的点添加的elt运动轨迹点当中
    int nLen = int(trackDeque.size());

    double dSrcData0[nLen], dSrcData1[nLen], dSrcData2[nLen], dSrcData3[nLen], dSrcData4[nLen], dSrcData5[nLen];
    double dDesData0[nLen], dDesData1[nLen], dDesData2[nLen], dDesData3[nLen], dDesData4[nLen], dDesData5[nLen];

    memset(dSrcData0, 0, sizeof(dSrcData0));
    memset(dSrcData1, 0, sizeof(dSrcData0));
    memset(dSrcData2, 0, sizeof(dSrcData0));
    memset(dSrcData3, 0, sizeof(dSrcData0));
    memset(dSrcData4, 0, sizeof(dSrcData0));
    memset(dSrcData5, 0, sizeof(dSrcData0));

    memset(dDesData0, 0, sizeof(dDesData0));
    memset(dDesData1, 0, sizeof(dDesData0));
    memset(dDesData2, 0, sizeof(dDesData0));
    memset(dDesData3, 0, sizeof(dDesData0));
    memset(dDesData4, 0, sizeof(dDesData0));
    memset(dDesData5, 0, sizeof(dDesData0));

    int nRow = 0;
    //根据方向，调用elt库函数，将队列中的点添加的elt运动轨迹点当中
    while(!trackDeque.empty())
    {
        EltPos targetPosTemp;
        if(nDirection == FORWARD)
        {
            targetPosTemp= trackDeque.front();
            trackDeque.pop_front();
        }
        else
        {
            targetPosTemp = trackDeque.back();
            trackDeque.pop_back();
        }

        dSrcData0[nRow] = targetPosTemp.eltPos[0];
        dSrcData1[nRow] = targetPosTemp.eltPos[1];
        dSrcData2[nRow] = targetPosTemp.eltPos[2];
        dSrcData3[nRow] = targetPosTemp.eltPos[3];
        dSrcData4[nRow] = targetPosTemp.eltPos[4];
        dSrcData5[nRow] = targetPosTemp.eltPos[5];

        nRow++;
    }

    LinearSmooth7(dSrcData0, dDesData0, nLen);
    LinearSmooth7(dSrcData1, dDesData1, nLen);
    LinearSmooth7(dSrcData2, dDesData2, nLen);
    LinearSmooth7(dSrcData3, dDesData3, nLen);
    LinearSmooth7(dSrcData4, dDesData4, nLen);
    LinearSmooth7(dSrcData5, dDesData5, nLen);

    for(int i=0; i<nLen; i++)
    {
        EltPos targetPosTemp;

        targetPosTemp.eltPos[0] = dDesData0[i];
        targetPosTemp.eltPos[1] = dDesData1[i];
        targetPosTemp.eltPos[2] = dDesData2[i];
        targetPosTemp.eltPos[3] = dDesData3[i];
        targetPosTemp.eltPos[4] = dDesData4[i];
        targetPosTemp.eltPos[5] = dDesData5[i];

        ret = elt_add_waypoint( m_eltCtx, targetPosTemp.eltPos, &err);
        if (ret != ELT_SUCCESS)
        {
            sErrMsg = "add way point err";
            ROS_INFO("[EliteRunDragTrack] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
            return -1;
        }
    }

    //执行轨迹
    int nMoveType = 0;
    ret = elt_track_move( m_eltCtx, nMoveType, m_nSmoothnessLevel, &err);

    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "elt track move error";
        ROS_INFO("[EliteRunDragTrack] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }
    return 1;
}

/*************************************************
Function: CEliteControl::ResetToOrigin
Description: 有报警则清除报警后，返回原点位置
Input: string &sOutput,反馈信息
Output: true,成功
        false,失败
Others: void
**************************************************/
bool CEliteControl::ResetToOrigin(string &sOutput)
{
    ROS_INFO("[ResetToOrigin] start.");
    m_nTaskName = RESET;

    if(m_nEliteState == ALARM)
    {
        //清除报警，3次重试。
        if(EliteClearAlarm() == -1)
        {
            sOutput.append("elt clear alarm error");
            ROS_ERROR("[ResetToOrigin]%s",sOutput.c_str());
            m_nTaskName = IDLE;
            return false;
        }

        if(EliteSyncMotorStatus() == -1)
        {
            sOutput.append("sync elite motor status failed");
            ROS_ERROR("[ResetToOrigin]%s",sOutput.c_str());
            m_nTaskName = IDLE;
            return false;
        }

        if(EliteOpenServo() == -1)
        {
            sOutput.append("open elite servo failed");
            ROS_ERROR("[ResetToOrigin]%s",sOutput.c_str());
            m_nTaskName = IDLE;
            return false;
        }
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if(m_bRecordDragTrack)
    {
        if(EliteDrag(DISABLE) == -1)
        {
            sOutput = "disable drag failed";
            ROS_ERROR("[ResetToOrigin]%s",sOutput.c_str());
            m_nTaskName = IDLE;
            return false;
        }
        if(m_TrackFile.CloseFile() == -1)
        {
            sOutput = "close file failed";
            ROS_ERROR("[ResetToOrigin]%s",sOutput.c_str());
            m_nTaskName = IDLE;
            return false;
        }
        ROS_INFO("[ResetToOrigin] disable drag and close the orbit file");
    }

    if(!EliteGotoOrigin(sOutput))
    {
        ROS_ERROR("[ResetToOrigin]%s",sOutput.c_str());
        m_nTaskName = IDLE;
        return false;
    }

    this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

/*************************************************
Function: CEliteControl::EliteGotoOrigin
Description: 返回原点位置
Input: string &sInput,轨迹全路径+文件文件名 可为空
       string &sOutput,反馈信息
Output: true,成功
        false,失败
Others: void
**************************************************/
bool CEliteControl::EliteGotoOrigin(string &sOutput)
{
    ROS_INFO("[EliteGotoOrigin] start.");

    if(CheckOrigin(m_EliteCurrentPos))
    {
        ROS_INFO("[EliteGotoOrigin] robot arm is already at origin.");
        return true;
    }
    m_bIsRecordReset = false;

    std::unique_lock<std::mutex> Lock(m_ResetFileMutex);
    if(m_ResetFile.CloseFile() == -1)
    {
        ROS_ERROR("[EliteGotoOrigin] close reset file error:%s", m_sResetFile.c_str());
        UnInit();
        exit(-1);
    }

    if(EliteRunDragTrack(m_sResetFile, m_dEltSpeed, REVERSE, sOutput) == -1)
    {
        if(sOutput == "empty")
        {
            sOutput = "";
            if(EliteMultiPointMove(m_EltOriginPos, m_dEltSpeed, sOutput) == -1)
            {
                ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
                return false;
            }

            if(!WaitForMotionStop(m_nTimeoutLen, sOutput))
            {
                ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
                return false;
            }
            if(!CheckOrigin(m_EliteCurrentPos))
            {
                sOutput = "reset motion done, but did not reach the origin";
                ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
            return false;
        }

    }


    if(!WaitForMotionStop(m_nTimeoutLen, sOutput))
    {
        ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
        return false;
    }
    if(!CheckOrigin(m_EliteCurrentPos))
    {
        sOutput = "reset motion done, but did not reach the origin";
        ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
        return false;
    }
    else
    {
        //清空复位轨迹文件
        if(m_ResetFile.OpenFile(m_sResetFile, O_CREAT | O_TRUNC | O_RDWR) == -1)
        {
            ROS_ERROR("[EliteGotoOrigin] open reset file error:%s", m_sResetFile.c_str());
            return false;
        }
        m_bWriteOrigin = true;
    }

    Lock.unlock();

    return true;
}

/*************************************************
Function: CEliteControl::EliteSyncMotorStatus
Description: 同步机械臂数据
Input: bool bSwitchStatusFirst,默认为true,是否首先进行一次同步后再进行判断
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteSyncMotorStatus(bool bSwitchStatusFirst)
{
    //获取同步状态,返回1，表示机器人处于同步状态。 返回0，表示机器人处于未同步状态。
    elt_error err;
    int nRet, nPowerStatus = ELT_FALSE, nRetryTimes = 0;

    //同步伺服编码器数据
    if(bSwitchStatusFirst)
    {
        ROS_INFO("[EliteSyncMotorStatus] sync elt");
        nRet = elt_sync_motor_status(m_eltCtx, &err);

        if (ELT_SUCCESS != nRet)
        {
            ROS_INFO("[EliteSyncMotorStatus] sync motor failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }

        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    elt_get_motor_status(m_eltCtx, &nPowerStatus, &err);

    while(nPowerStatus == ELT_FALSE && nRetryTimes < 3)
    {
        //同步伺服编码器数据
        ROS_INFO("[EliteSyncMotorStatus] sync elt");
        nRet = elt_sync_motor_status(m_eltCtx, &err);

        if (ELT_SUCCESS != nRet)
        {
            ROS_INFO("[EliteSyncMotorStatus] sync motor failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }
        this_thread::sleep_for(std::chrono::milliseconds(1000));

        nRet = elt_get_motor_status(m_eltCtx, &nPowerStatus, &err);
        if (ELT_SUCCESS != nRet)
        {
            ROS_ERROR("[EliteSyncMotorStatus] get motor status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }

        ROS_INFO("[EliteSyncMotorStatus] nPowerStatus = %d",nPowerStatus);
        nRetryTimes++;
    }

    if(nPowerStatus == ELT_FALSE)
    {
        ROS_ERROR("[EliteSyncMotorStatus] sync elt failed");
        return -1;
    }

    return 1;
}

/*************************************************
Function: CEliteControl::EliteOpenServo
Description: 打开机械臂伺服
Input: void
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteOpenServo()
{
    elt_error err;
    int nRet, nServoStatus = ELT_FALSE, nRetryTimes = 0;

    // 返回1，表示伺服为打开状态。 返回0，表示伺服为关闭状态。
    nRet = elt_get_servo_status(m_eltCtx, &nServoStatus, &err);
    if (ELT_SUCCESS != nRet)
    {
        ROS_ERROR("[EliteOpenServo] get elt servo status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
        return -1;
    }

    while(nServoStatus == ELT_FALSE && nRetryTimes < 3)
    {
        //同步伺服编码器数据
        nRet = elt_set_servo_status(m_eltCtx, ENABLE, &err);

        if (ELT_SUCCESS != nRet)
        {
            ROS_INFO("[EliteOpenServo] set elt servo failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }
        this_thread::sleep_for(std::chrono::milliseconds(1000));

        nRet = elt_get_servo_status(m_eltCtx, &nServoStatus, &err);
        if (ELT_SUCCESS != nRet)
        {
            ROS_ERROR("[EliteOpenServo] get elt servo status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }

        ROS_INFO("[EliteOpenServo] nServoStatus = %d",nServoStatus);
        nRetryTimes++;
    }

    if(ELT_FALSE  == nServoStatus)
    {
        ROS_ERROR("[Init] get elt servo status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
        return -1;
    }

    return 1;
}

/*************************************************
Function: CEliteControl::EliteClearAlarm
Description: 清除机械臂告警状态,3次重试
Input: void
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteClearAlarm()
{
    //获取机械臂当前状态
    elt_error err;
    int nRet, nEltStatus, nRetryTimes = 0;

    // 0：停止状态，1：暂停状态，2：急停状态，3：运行状态，4：报警状态
    nRet = elt_get_robot_state(m_eltCtx, &nEltStatus, &err);
    ROS_INFO("[EliteClearAlarm] elt state: %d", nRet);

    while(nEltStatus != STOP && nRetryTimes < 3)
    {
        nRet = elt_clear_alarm(m_eltCtx, 1, &err);
        if (ELT_SUCCESS != nRet)
        {
            ROS_ERROR("[EliteClearAlarm] clear alarm failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }
        this_thread::sleep_for(std::chrono::milliseconds(1000));

        nRet = elt_get_robot_state(m_eltCtx, &nEltStatus, &err);
        if (ELT_SUCCESS != nRet)
        {
            ROS_ERROR("[EliteClearAlarm] get state failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
            return -1;
        }

        ROS_INFO("[EliteClearAlarm] nEltStatus = %d",nEltStatus);
        nRetryTimes++;
    }
    if(nEltStatus != STOP)
    {
        ROS_ERROR("[EliteClearAlarm] clear alarm failed");
        return -1;
    }
    ROS_INFO("[EliteClearAlarm] elt status is normal");
    return 1;
}

/*************************************************
Function: CEliteControl::CheckOrigin
Description: 检测位置信息是否为原点位置
Input: elt_robot_pos &pos_array, 位置信息数组
Output: true or false
Others: void
**************************************************/
bool CEliteControl::CheckOrigin(elt_robot_pos &pos_array, bool bCheckBase)
{
    int nOrigin = 0;
    int nBaseOriginAxisNum = 3, nArmOriginAxisNum = 6;
    int nAxisNum = bCheckBase ? nBaseOriginAxisNum : nArmOriginAxisNum;
    for(int i=0; i<nAxisNum; i++)
    {
        if(abs(pos_array[i] - m_EltOriginPos[i]) < 0.1)
        {
            nOrigin++;
        }
    }

    return (bCheckBase ? nOrigin == nBaseOriginAxisNum : nOrigin == nArmOriginAxisNum);
}

/*************************************************
Function: CEliteControl::PrintJointData
Description: 打印关节角度信息
Input: elt_robot_pos &pos_array, 位置信息数组
        string sFunName, 调用者名称
Output: void
Others: void
**************************************************/
void CEliteControl::PrintJointData(elt_robot_pos &pos_array, string sFunName)
{
    string sPrintMsg = "["+sFunName+"][PrintJointData]";
    for(int i=0;i<6;i++)
    {
        sPrintMsg.append(" ").append(to_string(pos_array[i]));
    }
    ROS_INFO("%s",sPrintMsg.c_str());
}

/*************************************************
Function: CEliteControl::WaitForMotionStop
Description: 超时等待函数：等待机械臂运动结束
Input: int nTimeoutTime, 超时等待时间（单位：秒）
       string sErr, 存储false时的错误信息
Output: true or false
Others: void
**************************************************/
bool CEliteControl::WaitForMotionStop(int nTimeoutTime, string &sErr)
{
    this_thread::sleep_for(std::chrono::milliseconds(200));

    timespec m_tStartTime;
    timespec_get(&m_tStartTime, TIME_UTC);

    while(m_nEliteState == MOVING)
    {
        //机械臂设置 rool pitch yaw 超时检测
        this_thread::sleep_for(std::chrono::milliseconds(100));

        timespec CurrentTime;
        timespec_get(&CurrentTime, TIME_UTC);

        __time_t pollInterval = (CurrentTime.tv_sec - m_tStartTime.tv_sec) \
                                    +(CurrentTime.tv_nsec - m_tStartTime.tv_nsec) / 1000000000;
        if(pollInterval > nTimeoutTime)
        {
            sErr = "timeout error";
            ROS_WARN("[WaitForMotionStop]output:%s", sErr.c_str());
            return false;
        }
    }
    if(m_nEliteState != STOP)
    {
        sErr = "arm move error";
        ROS_WARN("[WaitForMotionStop]output:%s", sErr.c_str());
        return false;
    }

    return true;
}

/*************************************************
Function: CEliteControl::ArmServiceFunc
Description: service 功能函数
Input: wootion_msgs::ControlService::Request &Req, 请求
       wootion_msgs::ControlService::Response &Resp, 响应
Output: void
Others: void
**************************************************/
bool CEliteControl::ArmServiceFunc(wootion_msgs::ControlService::Request &Req, wootion_msgs::ControlService::Response &Resp)
{
    if (m_nCheckReceiver == 1 && Req.receiver != m_sNodeName)
    {
        ROS_INFO("[ArmServiceFunc] receiver is:%s, ignore",Req.receiver.c_str());
        return true;
    }

    Resp.trans_id = Req.trans_id;
    Resp.ack = "failed";
    Resp.data.clear();

    ROS_INFO("[ArmServiceFunc] sender:%s, request:%s, data:%s", Req.sender.c_str(), Req.cmd.c_str(), Req.data.c_str());

    if ((m_nEliteState == EMERGENCY_STOP || m_nEliteState == ALARM) &&
        Req.cmd != "reset" && Req.cmd != "record_orbit" && Req.cmd != "play_orbit")
    {
        ROS_ERROR("[ArmServiceFunc] work status abnormal");
        Resp.data = "work status abnormal";
        return true;
    }

    if (m_nEliteState == PAUSE && Req.cmd != "reset")
    {
        ROS_INFO("[ArmServiceFunc] robot arm pause, has unfinished work");
        Resp.data = "robot arm pause, has unfinished work";
        return true;
    }

    if (m_bBusy)
    {
        ROS_INFO("[ArmServiceFunc] robot arm is busy");
        Resp.data = "robot arm is busy";
        return true;
    }

    if (m_nEliteMode != Remote && \
            (Req.cmd == "record_orbit" || Req.cmd == "play_orbit" || Req.cmd == "rotate" ||\
             Req.cmd == "set_orientation" || Req.cmd == "set_yaw_angle" || Req.cmd == "set_position" || Req.cmd == "reset"))
    {
        Resp.data = "mode error,the mode of robot arm is ";
        m_nEliteMode == Teach ? Resp.data.append("teach") : Resp.data.append("play");
        ROS_INFO("[ArmServiceFunc] %s",Resp.data.c_str());
        return true;
    }

    if (m_nEliteState == MOVING && \
            (Req.cmd == "record_orbit" || Req.cmd == "play_orbit" || Req.cmd == "rotate" ||\
             Req.cmd == "set_orientation" || Req.cmd == "set_yaw_angle" || Req.cmd == "set_position" || Req.cmd == "reset"))
    {
        ROS_INFO("[ArmServiceFunc] robot arm is moving");
        Resp.data = "robot arm is moving";
        return true;
    }

    if (m_bRecordDragTrack && m_nEliteState != ALARM &&\
            (Req.cmd == "record_orbit" || Req.cmd == "play_orbit" || Req.cmd == "rotate" ||\
             Req.cmd == "set_orientation" || Req.cmd == "set_yaw_angle" || Req.cmd == "set_position" || Req.cmd == "reset"))
    {
        ROS_INFO("[ArmServiceFunc] robot arm is dragging");
        Resp.data = "robot arm is dragging";
        return true;
    }

    m_bBusy = true;

    Resp.ack = ArmOperation(Req.cmd, Req.data, Resp.data) ? "success" : "failed";

    m_bBusy = false;

    ROS_INFO("[ArmServiceFunc] response:%s, data:%s", Resp.ack.c_str(), Resp.data.c_str());
    return true;
}

/*************************************************
Function: CEliteControl::ArmCmdCallBack
Description: topic 回调函数
Input: wootion_msgs::GeneralCmd::ConstPtr &ArmCmd, 指令
Output: void
Others: void
**************************************************/
void CEliteControl::ArmCmdCallBack(const wootion_msgs::GeneralCmd::ConstPtr &ArmCmd)
{
    if (m_nCheckReceiver == 1 && ArmCmd->receiver != m_sNodeName)
    {
        return;
    }

    wootion_msgs::GeneralAck ArmAck;

    ROS_INFO("[ArmCmdCallBack] sender:%s, cmd:%s, data:%s", ArmCmd->sender.c_str(), ArmCmd->cmd.c_str(), ArmCmd->data.c_str());

    ArmAck.header.stamp = ros::Time::now();
    ArmAck.sender = "robot_arm";
    ArmAck.receiver = ArmCmd->sender;
    ArmAck.trans_id = ArmCmd->trans_id;
    ArmAck.ack = "failed";
    ArmAck.data.clear();

    if ((m_nEliteState == EMERGENCY_STOP || m_nEliteState == ALARM) && \
        ArmCmd->cmd != "reset" && ArmCmd->cmd != "record_orbit" && ArmCmd->cmd != "play_orbit")
    {
        ROS_ERROR("[ArmCmdCallBack] work status abnormal");
        ArmAck.data = "work status abnormal";
        m_ArmAckPublisher.publish(ArmAck);
        return;
    }

    if (m_nEliteState == PAUSE && ArmCmd->cmd != "reset")
    {
        ROS_INFO("[ArmCmdCallBack] robot arm pause, has unfinished work");
        ArmAck.data = "robot arm pause, has unfinished work";
        m_ArmAckPublisher.publish(ArmAck);
        return;
    }

    if (m_bBusy)
    {
        ROS_INFO("[ArmCmdCallBack] robot arm is busy");
        ArmAck.data = "robot arm is busy";
        m_ArmAckPublisher.publish(ArmAck);
        return;
    }

    if (m_nEliteMode != Remote && \
            (ArmCmd->cmd == "record_orbit" || ArmCmd->cmd == "play_orbit" || ArmCmd->cmd == "rotate" ||\
             ArmCmd->cmd == "set_orientation" || ArmCmd->cmd == "set_yaw_angle" || ArmCmd->cmd == "set_position" || ArmCmd->cmd == "reset"))
    {
        ArmAck.data = "mode error,the mode of robot arm is ";
        m_nEliteMode == Teach ? ArmAck.data.append("teach") : ArmAck.data.append("play");
        ROS_INFO("[ArmServiceFunc] %s",ArmAck.data.c_str());
        return;
    }

    if (m_nEliteState == MOVING && \
            (ArmCmd->cmd == "record_orbit" || ArmCmd->cmd == "play_orbit" || ArmCmd->cmd == "rotate" ||\
             ArmCmd->cmd == "set_orientation" || ArmCmd->cmd == "set_yaw_angle" || ArmCmd->cmd == "set_position" || ArmCmd->cmd == "reset"))
    {
        ROS_INFO("[ArmCmdCallBack] robot arm is moving");
        ArmAck.data = "robot arm is moving";
        m_ArmAckPublisher.publish(ArmAck);
        return;
    }

    if (m_bRecordDragTrack && m_nEliteState != ALARM &&\
            (ArmCmd->cmd == "record_orbit" || ArmCmd->cmd == "play_orbit" || ArmCmd->cmd == "rotate" ||\
             ArmCmd->cmd == "set_orientation" || ArmCmd->cmd == "set_yaw_angle" || ArmCmd->cmd == "set_position" || ArmCmd->cmd == "reset"))
    {
        ROS_INFO("[ArmCmdCallBack] robot arm is moving");
        ArmAck.data = "robot arm is moving";
        m_ArmAckPublisher.publish(ArmAck);
        return;
    }

    m_bBusy = true;

    ArmAck.ack = ArmOperation(ArmCmd->cmd, ArmCmd->data, ArmAck.data) ? "success" : "failed";


    m_bBusy = false;

    ROS_INFO("[ArmCmdCallBack] ack:%s, data:%s", ArmAck.ack.c_str(), ArmAck.data.c_str());
    m_ArmAckPublisher.publish(ArmAck);
}

/*************************************************
Function: CEliteControl::ArmOperation
Description: service 和 topic 的逻辑执行函数
Input: const std::string &sCommand, service 或者 topic中的指令cmd
       const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::ArmOperation(const std::string &sCommand, const std::string &sInput, std::string &sOutput)
{
    ROS_INFO("[ArmOperation] command:%s, input data:%s", sCommand.c_str(), sInput.c_str());
    sOutput = "";

    if(m_bRecordDragTrack && sCommand != "record_orbit" && sCommand != "stop_orbit" && sCommand != "reset")
    {
        sOutput = "recording the arm orbit";
        return false;
    }

    if(sCommand == "play_orbit" || sCommand == "rotate" || sCommand == "reset" || sCommand == "set_orientation" \
        || sCommand == "set_yaw_angle" || sCommand == "set_position" || sCommand == "turn_around")
    {
        if(m_nEliteState == STOP)
        {
            bool bSwitchStatusFirst = false;
            if(EliteSyncMotorStatus(bSwitchStatusFirst) == -1)
            {
                sOutput = " sync elite motor status failed";
                return false;
            }
        }
    }

    if(sCommand == "record_orbit")
    {
        return RecordOrbit(sOutput);
    }
    else if(sCommand == "stop_orbit")
    {
        return StopOrbit(sOutput);
    }
    else if(sCommand == "play_orbit")
    {
        return PlayOrbit(sInput, sOutput);
    }
    else if(sCommand == "rotate")
    {
        return Rotate(sInput, sOutput);
    }
    else if(sCommand == "stop_rotate")
    {
        return StopRotate(sOutput);
    }
    else if(sCommand == "get_orientation")
    {
        return GetOrientation(sOutput);
    }
    else if(sCommand == "set_orientation")
    {
        return SetOrientation(sInput, sOutput);
    }
    else if(sCommand == "move")
    {

    }
    else if(sCommand == "stop_move")
    {

    }
    else if(sCommand == "get_position")
    {
        return GetPosition(sOutput);
    }
    else if(sCommand == "set_position")
    {
        return SetPosition(sInput, sOutput);
    }
    else if(sCommand == "reset")
    {
        return Reset(sOutput);
    }
    else if(sCommand == "get_yaw_angle")
    {
        return GetYawAngle(sOutput);
    }
    else if(sCommand == "set_yaw_angle")
    {
        return SetYawAngle(sInput, sOutput);
    }
    //机械臂1轴旋转180,镜头朝向相对化工机器人调转180.
    else if(sCommand == "turn_around")
    {
        return TurnAround(sOutput);
    }
    //测试代码，运动到固定点
    else if (sCommand == "goto")
    {
        return GotoNewPos(sInput, sOutput);
    }
    //测试代码，模拟teach功能
    else if (sCommand == "teach")
    {
        return VirtualTeach(sInput, sOutput);
    }
    else if(sCommand == "stop_teach")
    {
        return StopRotate(sOutput);
    }
    else
    {
        ROS_ERROR("unknown command:%s", sCommand.c_str());
        sOutput = "unknown command:" + sCommand;
        return false;
    }
}

/*************************************************
Function: CEliteControl::RecordOrbit
Description: 拖拽轨迹记录功能函数
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::RecordOrbit(std::string &sOutput)
{
    m_bIgnoreMove = true;
    if(!ResetToOrigin(sOutput))
    {
        ROS_ERROR("[RecordOrbit]%s",sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }

    string sTrackFile = m_sArmTrackPath + m_sOrbitFileName;
    ROS_INFO("[RecordOrbit] the orbit file is:%s",sTrackFile.c_str());

    if(m_TrackFile.CloseFile() == -1)
    {
        ROS_ERROR("[RecordOrbit] close orbit file failed");
    }
    if(m_TrackFile.OpenFile(sTrackFile, O_CREAT | O_TRUNC | O_RDWR) == -1)
    {
        sOutput = "open file failed";
        ROS_ERROR("[RecordOrbit]%s",sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }
    if(EliteDrag(ENABLE) == -1)
    {
        sOutput = "enable drag failed";
        ROS_ERROR("[RecordOrbit]%s",sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }
    m_sResetOrbitFile = sTrackFile;
    sOutput = sTrackFile;
    m_nTaskName = RECORD;
    m_bIgnoreMove = false;

    return true;
}

/*************************************************
Function: CEliteControl::StopOrbit
Description: 停止轨迹拖拽记录
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::StopOrbit(std::string &sOutput)
{
    if(EliteDrag(DISABLE) == -1)
    {
        sOutput = "disable drag failed";
        ROS_ERROR("[StopOrbit]%s",sOutput.c_str());
        return false;
    }
    if(m_TrackFile.CloseFile() == -1)
    {
        sOutput = "close file failed";
        ROS_ERROR("[StopOrbit]%s",sOutput.c_str());
        return false;
    }

    m_nTaskName = IDLE;
    ROS_INFO("[StopOrbit] disable drag and close the orbit file");

    memcpy(m_RotateOriginPos, m_EliteCurrentPos, sizeof(m_RotateOriginPos));
    ROS_INFO("[StopOrbit] update rotate origin\n");

    return true;
}

/*************************************************
Function: CEliteControl::PlayOrbit
Description: 播放拖拽记录的轨迹
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::PlayOrbit(const std::string &sInput, std::string &sOutput)
{
    string sTrackFile, sPlayFirstAxis;
    vector<string> vCmdList;
    boost::split(vCmdList, sInput, boost::is_any_of(","), boost::token_compress_on);
    if(vCmdList.size() != 2)
    {
        sOutput = "input error" + sInput;
        ROS_ERROR("[PlayOrbit]%s",sOutput.c_str());
        return false;
    }

    sTrackFile = vCmdList[0];
    sPlayFirstAxis = vCmdList[1];

    if(sPlayFirstAxis == "0")
    {
        int nOrigin = 0;
        elt_robot_pos CurrentPos;
        memcpy(CurrentPos, m_EliteCurrentPos, sizeof(CurrentPos));
        for(int i=1; i<6; i++)
        {
            if(abs(CurrentPos[i] - m_EltOriginPos[i]) < 0.1)
            {
                nOrigin++;
            }
        }

        if(nOrigin != 5)
        {
            sOutput = "play orbit excepting 1st axis, the other axis is not at origin";
            ROS_ERROR("[PlayOrbit]%s",sOutput.c_str());
            return false;
        }
    }
    else
    {
        m_bIgnoreMove = true;
        bool bHasReset = !CheckOrigin(m_EliteCurrentPos);
        if(!ResetToOrigin(sOutput))
        {
            ROS_ERROR("[PlayOrbit]%s",sOutput.c_str());
            m_bIgnoreMove = false;
            return false;
        }

        m_nTaskName = PLAY;
        if(bHasReset)
        {
            this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    
    m_nTaskName = PLAY;

    if(sTrackFile.empty() || sTrackFile == "null" || sTrackFile == " ")
    {
        sTrackFile = m_sArmTrackPath + m_sOrbitFileName;
    }

    if (access(sTrackFile.c_str(), 0))
    {
        sOutput = "orbit file is null";
        ROS_ERROR("[PlayOrbit]%s", sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }

    if (EliteRunDragTrack(sTrackFile, m_dEltSpeed, FORWARD, sOutput, sPlayFirstAxis) == -1)
    {
        ROS_ERROR("[PlayOrbit]%s", sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }

    if(!WaitForMotionStop(m_nTimeoutLen, sOutput))
    {
        m_bIgnoreMove = false;
        return false;
    }

    m_sResetOrbitFile = sTrackFile;
    m_bIgnoreMove = false;

    memcpy(m_RotateOriginPos, m_EliteCurrentPos, sizeof(m_RotateOriginPos));
    ROS_INFO("[PlayOrbit] update rotate origin\n");

    return true;
}

/*************************************************
Function: CEliteControl::Rotate
Description: 模拟云台的3轴运动
Input: const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::Rotate(const std::string &sInput, std::string &sOutput)
{
    std::string sYawAngle;
    std::string sPitchAngle;
    std::string sRollAngle;

    vector<string> vCmdList;
    boost::split(vCmdList, sInput, boost::is_any_of(","), boost::token_compress_on);

    if (vCmdList.size() < 4)
    {
        sOutput = "input error" + sInput;
        ROS_ERROR("[Rotate] %s",sOutput.c_str());
        return false;
    }

    sRollAngle = vCmdList[0];
    sPitchAngle = vCmdList[1];
    sYawAngle = vCmdList[2];

    int nSpeedLevel = stoi(vCmdList[3]);

    if(nSpeedLevel < 1 || nSpeedLevel > 63)
    {
        sOutput = "input speed level error" + sInput;
        ROS_ERROR("[Rotate] %s",sOutput.c_str());
        return false;
    }

    double dSpeed = (m_dRotateSpeed*stod(vCmdList[3]))/63;

    elt_robot_pos targetPos;
    memcpy(targetPos, m_EliteCurrentPos, sizeof(targetPos));

    if(sRollAngle == "+")
    {
        targetPos[AXIS_ROLL] = m_RotateOriginPos[AXIS_ROLL] + m_dRotateLimitAngle;
        if(targetPos[AXIS_ROLL] > AxisLimitAngle[AXIS_SIX_MAX])
            targetPos[AXIS_ROLL] = AxisLimitAngle[AXIS_SIX_MAX];
    }
    else if(sRollAngle == "-")
    {
        targetPos[AXIS_ROLL] = m_RotateOriginPos[AXIS_ROLL] - m_dRotateLimitAngle;
        if(targetPos[AXIS_ROLL] < AxisLimitAngle[AXIS_SIX_MIN])
            targetPos[AXIS_ROLL] = AxisLimitAngle[AXIS_SIX_MIN];
    }

    if(sPitchAngle == "+")
    {
        targetPos[AXIS_PITCH] = m_RotateOriginPos[AXIS_PITCH] + m_dRotateLimitAngle;
        if(targetPos[AXIS_PITCH] > AxisLimitAngle[AXIS_FOUR_MAX])
            targetPos[AXIS_PITCH] = AxisLimitAngle[AXIS_FOUR_MAX];
    }
    else if(sPitchAngle == "-")
    {
        targetPos[AXIS_PITCH] = m_RotateOriginPos[AXIS_PITCH] - m_dRotateLimitAngle;
        if(targetPos[AXIS_PITCH] < AxisLimitAngle[AXIS_FOUR_MIN])
            targetPos[AXIS_PITCH] = AxisLimitAngle[AXIS_FOUR_MIN];
    }

    if(sYawAngle == "-")
    {
        targetPos[AXIS_YAW] = m_RotateOriginPos[AXIS_YAW] + m_dRotateLimitAngle;
        if(targetPos[AXIS_YAW] > AxisLimitAngle[AXIS_FIVE_MAX])
            targetPos[AXIS_YAW] = AxisLimitAngle[AXIS_FIVE_MAX];
    }
    else if(sYawAngle == "+")
    {
        targetPos[AXIS_YAW] = m_RotateOriginPos[AXIS_YAW] - m_dRotateLimitAngle;
        if(targetPos[AXIS_YAW] < AxisLimitAngle[AXIS_FIVE_MIN])
            targetPos[AXIS_YAW] = AxisLimitAngle[AXIS_FIVE_MIN];
    }

    if(EliteJointMove(targetPos, dSpeed, sOutput) == -1)
    {
        sOutput.append(",rotate failed");
        ROS_ERROR("[Rotate]%s",sOutput.c_str());
        return false;
    }

    m_nTaskName = ROTATE;

    return true;
}

/*************************************************
Function: CEliteControl::StopRotate
Description: 模拟云台的3轴运动停止
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::StopRotate(std::string &sOutput)
{
    if(EliteStop(sOutput) == -1)
    {
        sOutput.append(",stop - failed");
        ROS_ERROR("[StopRotate]%s",sOutput.c_str());
        return false;
    }

    return true;
}

/*************************************************
Function: CEliteControl::GetOrientation
Description: 获取模拟云台3个轴的绝对位置角度信息
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::GetOrientation(std::string &sOutput)
{
    sOutput.append(to_string(int(round(m_EliteCurrentPos[AXIS_ROLL]*100))));
    sOutput.append(",").append(to_string(int(round(m_EliteCurrentPos[AXIS_PITCH]*100))));
    sOutput.append(",").append(to_string(int(round(m_EliteCurrentPos[AXIS_YAW]*100))));

    return true;
}

/*************************************************
Function: CEliteControl::SetOrientation
Description: 设置模拟云台3个轴的绝对位置角度信息
Input: const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::SetOrientation(const std::string &sInput, std::string &sOutput)
{
    vector<string> vCmdList;
    boost::split(vCmdList, sInput, boost::is_any_of(","), boost::token_compress_on);

    if (vCmdList.size() < 3)
    {
        sOutput = "input error" + sInput;
        ROS_ERROR("[SetOrientation]%s",sOutput.c_str());
        return false;
    }

    elt_robot_pos targetPos;
    memcpy(targetPos, m_EliteCurrentPos, sizeof(targetPos));

    targetPos[AXIS_ROLL] = stod(vCmdList[0])/100;
    targetPos[AXIS_PITCH] = stod(vCmdList[1])/100;
    targetPos[AXIS_YAW] = stod(vCmdList[2])/100;

    if(abs(targetPos[AXIS_ROLL] - m_RotateOriginPos[AXIS_ROLL]) > m_dRotateLimitAngle)
    {
        double dOriginAngle = m_RotateOriginPos[AXIS_ROLL];
        ROS_WARN("[SetOrientation] roll angle over limit,target roll=%f, origin roll=%f",targetPos[AXIS_ROLL], dOriginAngle);
        targetPos[AXIS_ROLL] = ((targetPos[AXIS_ROLL] - dOriginAngle) > 0) ? dOriginAngle + m_dRotateLimitAngle : dOriginAngle - m_dRotateLimitAngle;
    }
    if(abs(targetPos[AXIS_PITCH] - m_RotateOriginPos[AXIS_PITCH]) > m_dRotateLimitAngle)
    {
        double dOriginAngle = m_RotateOriginPos[AXIS_PITCH];
        ROS_WARN("[SetOrientation] pitch angle over limit,target pitch=%f, origin pitch=%f",targetPos[AXIS_PITCH], dOriginAngle);
        targetPos[AXIS_PITCH] = ((targetPos[AXIS_PITCH] - dOriginAngle) > 0) ? dOriginAngle + m_dRotateLimitAngle : dOriginAngle - m_dRotateLimitAngle;
    }
    if(abs(targetPos[AXIS_YAW] - m_RotateOriginPos[AXIS_YAW]) > m_dRotateLimitAngle)
    {
        double dOriginAngle = m_RotateOriginPos[AXIS_YAW];
        ROS_WARN("[SetOrientation] yaw angle over limit,target yaw=%f, origin yaw=%f",targetPos[AXIS_YAW], dOriginAngle);
        targetPos[AXIS_YAW] = ((targetPos[AXIS_YAW] - dOriginAngle) > 0) ? dOriginAngle + m_dRotateLimitAngle : dOriginAngle - m_dRotateLimitAngle;
    }

    if(EliteMultiPointMove(targetPos, m_dEltSpeed, sOutput) == -1)
    {
        sOutput.append(",roll - failed");
        ROS_ERROR("[SetOrientation]%s",sOutput.c_str());
        return false;
    }

    m_nTaskName = ROTATE;

    return WaitForMotionStop(m_nTimeoutLen, sOutput);
}

/*************************************************
Function: CEliteControl::GetPosition
Description: 获取末端位置信息
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::GetPosition(std::string &sOutput)
{
    elt_robot_pose response_pose_array;
    elt_error err;
    if(elt_positive_kinematic(m_eltCtx, m_EliteCurrentPos, response_pose_array, &err) == -1)
    {
        sOutput = "get position error:";
        sOutput.append(err.err_msg);
        ROS_ERROR("[GetPosition]%s",sOutput.c_str());
        return false;
    }

    sOutput = to_string(response_pose_array[0]);
    sOutput.append(",").append(to_string(response_pose_array[1]));
    sOutput.append(",").append(to_string(response_pose_array[2]));

    return true;
}

/*************************************************
Function: CEliteControl::SetPosition
Description: 设置位置
Input: const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::SetPosition(const std::string &sInput, std::string &sOutput)
{
    std::string sPosX;
    std::string sPosY;
    std::string sPosZ;

    vector<string> vCmdList;
    boost::split(vCmdList, sInput, boost::is_any_of(","), boost::token_compress_on);

    if (vCmdList.size() < 3)
    {
        sOutput = "input error" + sInput;
        ROS_ERROR("[SetPosition]%s",sOutput.c_str());
        return false;
    }

    sPosX = vCmdList[0];
    sPosY = vCmdList[1];
    sPosZ = vCmdList[2];

    elt_robot_pose ElitePose;
    memset(ElitePose, 0, sizeof(ElitePose));

    ElitePose[0] = stod(sPosX);
    ElitePose[1] = stod(sPosY);
    ElitePose[2] = stod(sPosZ);

    elt_robot_pos targetPos;
    elt_error err;
    if(elt_inverse_kinematic(m_eltCtx, ElitePose, targetPos, &err) == -1)
    {
        sOutput = "set position error:";
        sOutput.append(err.err_msg);
        ROS_ERROR("[SetPosition]%s",sOutput.c_str());
        return false;
    }

    if(EliteMultiPointMove(targetPos, m_dEltSpeed, sOutput) == -1)
    {
        ROS_ERROR("[SetPosition]%s",sOutput.c_str());
        return false;
    }
    m_nTaskName = ROTATE;

    return true;
}

/*************************************************
Function: CEliteControl::Reset
Description: 回到自定义原点位置
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::Reset(std::string &sOutput)
{
    m_nTaskName = RESET;
    m_bIgnoreMove = true;

    //清除碰撞报警所引入的暂停状态
    if(m_nEliteState == PAUSE)
    {
        if(!EliteStop(sOutput))
        {
            ROS_ERROR("[Reset]%s",sOutput.c_str());
            m_bIgnoreMove = false;
            return false;
        }

        timespec m_tStartTime;
        timespec_get(&m_tStartTime, TIME_UTC);

        while(m_nEliteState != STOP)
        {
            //机械臂执行拖拽轨迹超时检测
            this_thread::sleep_for(std::chrono::milliseconds(1000));

            timespec CurrentTime;
            timespec_get(&CurrentTime, TIME_UTC);

            __time_t pollInterval = (CurrentTime.tv_sec - m_tStartTime.tv_sec) \
                                    +(CurrentTime.tv_nsec - m_tStartTime.tv_nsec) / 1000000000;
            if(pollInterval > m_nTimeoutLen)
            {
                sOutput = "reset collision alarm timeout error";
                ROS_WARN("[Reset] %s", sOutput.c_str());
                m_bIgnoreMove = false;
                return false;
            }
        }
    }

    if(!ResetToOrigin(sOutput))
    {
        ROS_ERROR("[Reset]%s",sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }
    m_sResetOrbitFile = "null";
    m_bIgnoreMove = false;

    if(m_nDragStatus == ENABLE)
    {
        m_nDragStatus = DISABLE;
    }

    memcpy(m_RotateOriginPos, m_EliteCurrentPos, sizeof(m_RotateOriginPos));
    ROS_INFO("[Reset] update rotate origin\n");

    return true;
}

/*************************************************
Function: CEliteControl::GetYawAngle
Description: 获取机械臂第1个轴的绝对位置角度信息
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::GetYawAngle(std::string &sOutput)
{
    sOutput.append(to_string(int(round(m_EliteCurrentPos[0]*100))));
    return true;
}

/*************************************************
Function: CEliteControl::SetYawAngle
Description: 设置机械臂第1个轴的绝对位置角度信息
Input: const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::SetYawAngle(const std::string &sInput, std::string &sOutput)
{
    m_bIgnoreMove = true;
    bool bHasReset = false;

    int nOrigin = 0;
    elt_robot_pos CurrentPos;
    memcpy(CurrentPos, m_EliteCurrentPos, sizeof(CurrentPos));
    for(int i=1; i<6; i++)
    {
        if(abs(CurrentPos[i] - m_EltOriginPos[i]) < 0.1)
        {
            nOrigin++;
        }
    }

    if(nOrigin != 5)
    {
        ROS_ERROR("[PlayOrbit] set yaw angle, excepting 1st axis, the other axis is not at origin");
        if(!ResetToOrigin(sOutput))
        {
            ROS_ERROR("[SetYawAngle]%s",sOutput.c_str());
            m_bIgnoreMove = false;
            return false;
        }
        bHasReset = true;
    }

    m_nTaskName = SET_ARM_YAW;
    if(bHasReset)
    {
        this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    elt_robot_pos targetPos;
    memcpy(targetPos, m_EliteCurrentPos, sizeof(targetPos));

    targetPos[0] = stod(sInput)/100;

    if(EliteMultiPointMove(targetPos, m_dEltSpeed, sOutput) == -1)
    {
        sOutput.append(", set_yaw_angle failed");
        ROS_ERROR("[SetYawAngle]%s",sOutput.c_str());
        return false;
    }

    if(!WaitForMotionStop(m_nTimeoutLen, sOutput))
    {
        m_bIgnoreMove = false;
        return false;
    }

    m_bIgnoreMove = false;
    return true;
}

/*************************************************
Function: CEliteControl::TurnAround
Description: 一轴旋转180度
Input: std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::TurnAround(std::string &sOutput)
{
    if(!CheckOrigin(m_EliteCurrentPos))
    {
        sOutput = "robot arm is not at the origin";
        ROS_ERROR("[TurnAround]%s",sOutput.c_str());
        return false;
    }
    m_nTaskName = TURN_AROUND;
    m_bIgnoreMove = true;

    elt_robot_pos targetPos;
    memcpy(targetPos, m_EltOriginPos, sizeof(targetPos));

    targetPos[0] = targetPos[0]+180 > AxisLimitAngle[0] ? targetPos[0]-180 : targetPos[0]+180;

    if(EliteMultiPointMove(targetPos, m_dEltSpeed, sOutput) == -1)
    {
        sOutput.append(",turn around failed");
        ROS_ERROR("[TurnAround]%s",sOutput.c_str());
        m_bIgnoreMove = false;
        return false;
    }

    if(!WaitForMotionStop(m_nTimeoutLen, sOutput))
    {
        m_bIgnoreMove = false;
        return false;
    }

    m_bIgnoreMove = false;

    return true;
}

/*************************************************
Function: CEliteControl::GotoNewPos
Description: 测试函数,运动到某个节点坐标，或者直接坐标位置
Input: const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::GotoNewPos(const std::string &sInput, std::string &sOutput)
{
    vector<string> vCmdList;
    boost::split(vCmdList, sInput, boost::is_any_of(","), boost::token_compress_on);

    if (vCmdList.size() < 7)
    {
        sOutput = "input error" + sInput;
        ROS_ERROR("[GotoNewPos]%s",sOutput.c_str());
        return false;
    }

    elt_robot_pos targetPos;
    if(vCmdList[6] == "pose")
    {
        elt_robot_pose ElitePose;
        memset(ElitePose, 0, sizeof(ElitePose));
        elt_error err;

        for(int i=0; i<ROBOT_POSE_SIZE; i++)
        {
            if(i<3)
                ElitePose[i] = stod(vCmdList[i]);
            else
                ElitePose[i] = stod(vCmdList[i])/180*M_PI;
        }
        if(elt_inverse_kinematic(m_eltCtx, ElitePose, targetPos, &err) == -1)
        {
            sOutput = "set position error:";
            sOutput.append(err.err_msg);
            ROS_ERROR("[GotoNewPos]%s",sOutput.c_str());
            return false;
        }
    }
    else
    {
        for(int i=0; i<ROBOT_POSE_SIZE; i++)
        {
            targetPos[i] = stod(vCmdList[i]);
        }
    }

    if(EliteMultiPointMove(targetPos, m_dEltSpeed, sOutput) == -1)
    {
        ROS_ERROR("[GotoNewPos]%s",sOutput.c_str());
        return false;
    }

    return true;
}

/*************************************************
Function: CEliteControl::VirtualTeach
Description: 模拟teach模式
Input: const std::string &sInput, service 或者 topic中的指令对应的data
       std::string &sOutput, 处理结果反馈
Output: true 成功
        false 失败
Others: void
**************************************************/
bool CEliteControl::VirtualTeach(const std::string &sInput, std::string &sOutput)
{
    vector<string> vCmdList;
    boost::split(vCmdList, sInput, boost::is_any_of(","), boost::token_compress_on);

    if (vCmdList.size() < 3)
    {
        sOutput = "input error" + sInput;
        ROS_ERROR("[VirtualTeach] %s",sOutput.c_str());
        return false;
    }

    int nAxisNum = stoi(vCmdList[0]);
    std::string sDirection = vCmdList[1];
    int nSpeedLevel = stoi(vCmdList[2]);

    if(nAxisNum < 1 || nAxisNum > 6)
    {
        sOutput = "input axis num error" + sInput;
        ROS_ERROR("[VirtualTeach] %s",sOutput.c_str());
        return false;
    }

    if(nSpeedLevel < 1 || nSpeedLevel > 63)
    {
        sOutput = "input speed level error" + sInput;
        ROS_ERROR("[VirtualTeach] %s",sOutput.c_str());
        return false;
    }

    double dSpeed = (m_dEltSpeed*stod(vCmdList[2]))/63;

    elt_robot_pos targetPos;
    memcpy(targetPos, m_EliteCurrentPos, sizeof(targetPos));

    targetPos[nAxisNum-1] = sDirection == "+" ? AxisLimitAngle[nAxisNum-1] : AxisLimitAngle[nAxisNum-1+6];

    if(EliteJointMove(targetPos, dSpeed, sOutput) == -1)
    {
        sOutput.append(",teach - failed");
        ROS_ERROR("[VirtualTeach]%s",sOutput.c_str());
        return false;
    }

    m_nTaskName = TEACH;

    return true;
}

/*************************************************
Function: CEliteControl::AgvStatusCallBack
Description: topic 回调函数
Input: wootion_msgs::RobotStatus::ConstPtr &AgvStatus, 状态消息
Output: void
Others: void
**************************************************/
void CEliteControl::AgvStatusCallBack(const wootion_msgs::RobotStatus::ConstPtr &AgvStatus)
{
    bool bEmeStop = ((AgvStatus->warn_status[3] & 0x01) > 0);
    if(m_bEmeStop && !bEmeStop)
    {
        //清除报警，3次重试。
        if(EliteClearAlarm() == -1)
        {
            ROS_ERROR("[AgvStatusCallBack] clear elite alarm failed");
            return;
        }

        //同步数据, 3次重试
        if(EliteSyncMotorStatus() == -1)
        {
            ROS_ERROR("[AgvStatusCallBack] sync elite motor status failed");
            return;
        }

        //打开伺服,3次重试
        if(EliteOpenServo() == -1)
        {
            ROS_ERROR("[AgvStatusCallBack] open elite servo failed");
            return;
        }

        if(!m_bArmInit)
        {
            string sOutput;
            if(!ResetToOrigin(sOutput))
            {
                ROS_ERROR("[AgvStatusCallBack] %s",sOutput.c_str());
                return;
            }
            memcpy(m_RotateOriginPos, m_EliteCurrentPos, sizeof(m_RotateOriginPos));
            m_bArmInit = true;
        }

        if(m_nDragStatus == ENABLE)
        {
            m_nDragStatus = DISABLE;
        }
    }
    m_bEmeStop = bEmeStop;
}

/*************************************************
Function: CEliteControl::LinearSmooth7
Description: 数据光滑处理
Input: double *dSrc,源数组首地址
       double *dDest,目的数组首地址
       int nLen,数组总长度
Output: void
Others: void
**************************************************/
void CEliteControl::LinearSmooth7(double *dSrc, double *dDest, int nLen)
{
    int i;
    if ( nLen < 7 )
    {
        for ( i = 0; i <= nLen - 1; i++ )
        {
            dDest[i] = dSrc[i];
        }
    }
    else
    {
        dDest[0] = dSrc[0];

        dDest[1] = (5*dSrc[0] + 4*dSrc[1] + 3*dSrc[2] + 2*dSrc[3] + dSrc[4] - dSrc[6]) / 14.0;

        dDest[2] = (7*dSrc[0] + 6*dSrc [1] + 5*dSrc[2] + 4*dSrc[3] + 3*dSrc[4] + 2*dSrc[5] + dSrc[6]) / 28.0;

        for (i = 3; i <= nLen - 4; i++)
        {
            dDest[i] = (dSrc[i-3] + dSrc[i-2] + dSrc[i-1] + dSrc[i] + dSrc[i+1] + dSrc[i+2] + dSrc[i+3]) / 7.0;
        }

        dDest[nLen-3] = (7*dSrc[nLen-1] + 6*dSrc[nLen-2] + 5*dSrc[nLen-3] + 4*dSrc[nLen-4] + 3*dSrc[nLen-5] + \
                         2*dSrc[nLen-6] + dSrc[nLen-7] ) / 28.0;

        dDest[nLen-2] = (5*dSrc[nLen-1] + 4*dSrc[nLen-2] + 3*dSrc[nLen-3] + 2*dSrc[nLen-4] + dSrc[nLen-5] - \
                           dSrc[nLen-7] ) / 14.0;

        dDest[nLen-1] = dSrc[nLen-1];
    }
}

/*************************************************
Function: CEliteControl::CreateDataPath
Description: 创建轨迹数据的文件夹
Input: void
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::CreateDataPath()
{
    string sCmd = "mkdir -p ";
    sCmd.append(m_sArmTrackPath);
    int nStatus = system(sCmd.c_str());
    if(-1 == nStatus)
    {
        ROS_ERROR("[CreateDataPath] create data path failed, system error");
        return -1;
    }
    else
    {
        if(0 == int(WIFEXITED(nStatus)))
        {
            ROS_ERROR("[CreateDataPath] create data path failed, run shell error, exit code:%d", WIFEXITED(nStatus));
            return -1;
        }
    }
    return 1;
}

/*************************************************
Function: CEliteControl::RemoveErrPoints
Description: 机械臂bug,插入轨迹是会偶发出现异常的0.0 90.0 -90.0等异常点
Input: void
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
void CEliteControl::RemoveErrPoints(deque<EltPos> &trackDeque)
{
    deque<EltPos>::iterator iter;
    double dMinValue = 0.000001;
    int nCount = 0;

    for( iter = trackDeque.begin(); iter != trackDeque.end(); )
    {
        bool bErase = false;
        EltPos PreEltPos = (nCount==0) ? trackDeque[nCount] : trackDeque[nCount-1];
        EltPos NextEltPos = (nCount==trackDeque.size()-1) ? trackDeque[nCount] : trackDeque[nCount+1];

        for(int i=0; i<6; i++)
        {
            if(abs(iter->eltPos[i]) <= dMinValue || (abs(abs(iter->eltPos[i]) - 90.0) <= dMinValue))
            {
                if(abs(iter->eltPos[i] - PreEltPos.eltPos[i]) > m_dFilterValue || \
                   abs(iter->eltPos[i] - NextEltPos.eltPos[i]) > m_dFilterValue)
                {
                    bErase = true;
                    break;
                }
            }
        }

        if(bErase)
        {
            string sEraseData;
            for(int i=0; i<6; i++)
            {
                sEraseData.append(to_string(iter->eltPos[i])).append(" ");
            }
            ROS_WARN("[RemoveErrPoints] orbit exist error point:%s", sEraseData.c_str());
            trackDeque.erase(iter);
        }

        iter++;
        nCount++;
    }
}

CEliteControl::~CEliteControl()
{
    UnInit();
    m_pEliteStatusThread->join();
    m_pHeartBeatThread->join();
    m_pMonitorThread->join();

    delete m_pEliteStatusThread;
    delete m_pHeartBeatThread;
    delete m_pMonitorThread;
}