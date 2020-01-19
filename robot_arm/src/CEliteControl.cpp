//
// Created by zhoujg on 19-12-6.
//

#include "CEliteControl.h"

CEliteControl::CEliteControl():
    m_bRecordDragTrack(false),
    m_bBusy(false),
    m_nCheckReceiver(0),
    m_nDragStatus(DISABLE),
    m_nEliteState(ALARM),
    m_sOrbitFileName("arm_orbit.txt"),
    m_sTaskName("idle"),
    m_sResetOrbitFile("null")
{
    ros::NodeHandle PublicNodeHandle;
    ros::NodeHandle PrivateNodeHandle("~");

    PrivateNodeHandle.param("serial_port", m_sSerialPort, std::string("/dev/ttyS0"));
    PrivateNodeHandle.param("serial_speed", m_nSerialSpeed, 2400);
    PrivateNodeHandle.param("elite_ip", m_sEliteRobotIP, std::string("192.168.100.200"));
    PrivateNodeHandle.param("elite_port", m_nElitePort, 8055);
    PrivateNodeHandle.param("elt_speed", m_dEltSpeed, 10.0);
    PrivateNodeHandle.param("print_level", m_sPrintLevel, std::string("debug"));
    PrivateNodeHandle.param("track_path", m_sArmTrackPath, std::string(getenv("HOME")).append("/track/"));
    PublicNodeHandle.param("arm_service", m_sArmService, std::string("arm_control"));
    PublicNodeHandle.param("arm_cmd", m_sArmCmdTopic, std::string("arm_cmd"));
    PublicNodeHandle.param("arm_ack", m_sArmAckTopic, std::string("arm_ack"));
    PublicNodeHandle.param("arm_heart", m_sHeartBeatTopic, std::string("arm_heart_beat"));
    PublicNodeHandle.param("arm_abnormal", m_sAbnormalTopic, std::string("arm_exception"));

    ROS_INFO("[ros param] serial_port:%s", m_sSerialPort.c_str());
    ROS_INFO("[ros param] serial_speed:%d", m_nSerialSpeed);
    ROS_INFO("[ros param] elite_ip:%s", m_sEliteRobotIP.c_str());
    ROS_INFO("[ros param] elite_port:%d",m_nElitePort);
    ROS_INFO("[ros param] elt_speed:%f",m_dEltSpeed);
    ROS_INFO("[ros param] print_level:%s", m_sPrintLevel.c_str());
    ROS_INFO("[ros param] track_path:%s", m_sArmTrackPath.c_str());
    ROS_INFO("[ros param] arm_service:%s", m_sArmService.c_str());
    ROS_INFO("[ros param] arm_cmd:%s", m_sArmCmdTopic.c_str());
    ROS_INFO("[ros param] arm_ack:%s", m_sArmAckTopic.c_str());
    ROS_INFO("[ros param] arm_heart:%s", m_sHeartBeatTopic.c_str());
    ROS_INFO("[ros param] arm_heart:%s", m_sHeartBeatTopic.c_str());

    if(!Init())
    {
        ROS_ERROR("[CEliteControl] elite init failed");
        exit(-1);
    }

    m_tRecordDataTime.tv_sec = 0;
    m_tRecordDataTime.tv_nsec = 0;

    m_ArmService = PublicNodeHandle.advertiseService(m_sArmService, &CEliteControl::ArmServiceFunc, this);
    m_ArmCmdSubscriber = PublicNodeHandle.subscribe<wootion_msgs::GeneralCmd>(m_sArmCmdTopic, 10, boost::bind(&CEliteControl::ArmCmdCallBack, this, _1));
    m_ArmAckPublisher = PublicNodeHandle.advertise<wootion_msgs::GeneralAck>(m_sArmAckTopic, 10);
    m_HeartBeatPublisher = PublicNodeHandle.advertise<wootion_msgs::GeneralTopic>(m_sHeartBeatTopic, 10);
    m_AbnormalPublisher = PublicNodeHandle.advertise<wootion_msgs::GeneralTopic>(m_sAbnormalTopic, 10);

    this_thread::sleep_for(std::chrono::milliseconds(500));

    try
    {
        m_pEliteStatusThread = new std::thread(std::bind(&CEliteControl::UpdateEliteThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CEliteControl] malloc update-elite thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pHeartBeatThread = new std::thread(std::bind(&CEliteControl::HeartBeatThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CEliteControl] malloc heart beat thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pControlThread = new std::thread(std::bind(&CEliteControl::ControlThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CEliteControl] malloc motor control thread failed, %s", exception.what());
        exit(-1);
    }

    this_thread::sleep_for(std::chrono::milliseconds(500));

    string sTrackPath, sOutput;
    if(!EliteGotoOrigin(sTrackPath, sOutput))
    {
        ROS_ERROR("[CEliteControl] %s",sOutput.c_str());
    }
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
    elt_error err;
    int nRet = 0, nServoStatus = 1;
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

    //清除报警，3次重试。
    if(EliteClearAlarm() == -1)
    {
        ROS_ERROR("[Init] sync elite motor status failed");
        return false;
    }

    //获取同步状态,返回1，表示机器人处于同步状态。 返回0，表示机器人处于未同步状态。
    if(EliteSyncMotorStatus() == -1)
    {
        ROS_ERROR("[Init] sync elite motor status failed");
        return false;
    }

    //获取伺服状态
    // 返回1，表示伺服为打开状态。 返回0，表示伺服为关闭状态。
    nRet = elt_get_servo_status(m_eltCtx, &nServoStatus, &err);

    if (ELT_SUCCESS == nRet)
    {
        if(ELT_FALSE  == nServoStatus)
        {
            ROS_INFO("[Init] elt servo status is off.");

            //设置伺服状态
            ROS_INFO("[Init] turn elt servo on .");
            nRet = elt_set_servo_status(m_eltCtx, nServoStatus, &err);
            if (ELT_SUCCESS != nRet)
            {
                ROS_ERROR("[Init] set elt servo status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
                return false;
            }

            this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        ROS_INFO("[Init] servo status is normal");
    }
    else
    {
        ROS_ERROR("[Init] get elt servo status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
        return false;
    }

    //设置速度
    nRet = elt_set_waypoint_max_joint_speed( m_eltCtx, m_dEltSpeed,&err);
    if (ELT_SUCCESS != nRet)
    {
        ROS_ERROR("[Init] set elt joint speeds failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
        return false;
    }

    if (UpdateEltOrigin() == -1)
    {
        ROS_ERROR("[Init] UpdateEltOrigin error");
        return false;
    }

    if(!m_ExtraCtrl.Init(m_sSerialPort, m_nSerialSpeed))
    {
        ROS_ERROR("[Init] ExtraCtrl init error");
        return false;
    }

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
    int ret, nTimes = 0;
    elt_error err;
    elt_robot_pos EliteCurrentPos;
    memset(EliteCurrentPos, 0, sizeof(EliteCurrentPos));
    memset(&m_EliteCurrentPos, 0, sizeof(m_EliteCurrentPos));

    while(ros::ok())
    {
        //获取elt工作状态
        ret = elt_get_robot_state(m_eltCtx, &m_nEliteState, &err);
        if (ELT_SUCCESS != ret)
        {
            ROS_INFO("[UpdateEliteThreadFunc] get elite state failed");
        }

        //获取elt各个轴的绝对位置信息
        if(GetElitePos(m_EliteCurrentPos) == -1)
        {
            ROS_ERROR("[UpdateEliteThreadFunc] get elite current pos error");
        }

        //每200ms记录一次轨迹文件
        if(m_bRecordDragTrack && nTimes == 0)
        {
            //轨迹记录超时检测
            if(m_tRecordDataTime.tv_sec != 0 && m_tRecordDataTime.tv_nsec != 0)
            {
                timespec CurrentTime;
                timespec_get(&CurrentTime, TIME_UTC);

                __time_t pollInterval = (CurrentTime.tv_sec - m_tRecordDataTime.tv_sec) \
                                        +(CurrentTime.tv_nsec - m_tRecordDataTime.tv_nsec) / 1000000000;
                if(pollInterval > 180)
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

            //只有当角度差大于0.01时才将位置信息写入轨迹录制文件，剔除重复数据，剔除超限位数据
            string sAxisData;
            bool bIsWrite = false;
            for(int i=0; i<AXIS_COUNT; i++)
            {
                if((abs(m_EliteCurrentPos[i] - EliteCurrentPos[i]) > 0.1) &&\
                    m_EliteCurrentPos[i] <= AxisLimitAngle[i] && m_EliteCurrentPos[i] >= AxisLimitAngle[i+6])
                {
                    bIsWrite = true;
                }

                sAxisData.append(to_string(m_EliteCurrentPos[i]));
                if(i != AXIS_COUNT - 1)
                {
                    sAxisData.append(" ");
                }
            }
            if(bIsWrite)
            {
                sAxisData.append("\n");
                m_TrackFile.Output(sAxisData);
            }

            memcpy(EliteCurrentPos, m_EliteCurrentPos, sizeof(EliteCurrentPos));
        }

        //每200ms记录一次轨迹文件，改变周期可改变nTimes的变化阀值
        if(++nTimes == 5)
            nTimes = 0;

        this_thread::sleep_for(std::chrono::milliseconds(20));
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

    while(ros::ok())
    {
        //心跳消息
        wootion_msgs::GeneralTopic HeartBeatMsg;
        boost::property_tree::ptree ptAllItem;

        HeartBeatMsg.header.stamp = ros::Time::now();
        HeartBeatMsg.sender = "robot_arm";
        HeartBeatMsg.receiver = "";
        HeartBeatMsg.trans_id = 1;
        HeartBeatMsg.data.clear();

        ptAllItem.put("status", vsArmStatus[m_nEliteState]);
        if(m_nEliteState == STOP)
        {
            ptAllItem.put("task", "idle");
        }
        else
        {
            ptAllItem.put("task", m_sTaskName);
        }
        ptAllItem.put("drag", vsDragStatus[m_nDragStatus]);
        int nOrigin = 0;
        for(int i=0; i<6; i++)
        {
            if(abs(m_EliteCurrentPos[i] - m_EltOriginPos[i]) < 0.1)
            {
                nOrigin++;
            }
        }
        if(nOrigin == 6)
        {
            ptAllItem.put("origin", "true");
        }
        else
        {
            ptAllItem.put("origin", "false");
        }

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

        m_HeartBeatPublisher.publish(HeartBeatMsg);

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

        this_thread::sleep_for(std::chrono::seconds(1));
    }
}

/*************************************************
Function: CEliteControl::ControlThreadFunc
Description: 自测试线程函数
Input: void
Output: void
Others: void
**************************************************/
void CEliteControl::ControlThreadFunc()
{
    char c;
    while(ros::ok())
    {
        cin >> c;
        if(c == '\n')
            continue;

        if(c == 'q')
        {
            ROS_INFO("[ControlThreadFunc] process quit.");
            exit(-1);
        }
        if(c == 's')
        {
            ROS_INFO("[ControlThreadFunc] stop.");
            EliteStop();
        }
        if(c == 'e')
        {
            ROS_INFO("[ControlThreadFunc] enable elite drag.");
            elt_error err;
            int nRet = elt_set_servo_status(m_eltCtx, 0, &err);
            if (ELT_SUCCESS != nRet)
            {
                ROS_INFO("[Init] set elt servo status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
                continue;
            }
            ROS_INFO("[ControlThreadFunc] turn elt servo off .");

            this_thread::sleep_for(std::chrono::milliseconds(300));
            EliteDrag(ENABLE);
        }
        else if(c =='p')
        {
            PrintJointData(m_EliteCurrentPos,"m_EliteCurrentPos");
            ROS_INFO("[ControlThreadFunc]m_nEliteState=%d",m_nEliteState);
        }
        else if(c =='g')
        {
            ROS_INFO("[ControlThreadFunc] elt_run ");
            elt_error err;
            int nRet = elt_run(m_eltCtx , &err);
            if (ELT_SUCCESS != nRet)
            {
                ROS_INFO("[Init] elt_run failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
                continue;
            }
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(20));
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
    string sFileName = m_sArmTrackPath + "arm_origin.txt";
    ifstream originFile;

    //打开轨迹文件
    originFile.open(sFileName.c_str(), ios::out);
    if (!originFile.is_open())
    {
        ROS_ERROR("[EliteRunDragTrack] file cannot open,file:%s",sFileName.c_str());
        return -1;
    }

    while (!originFile.eof())
    {
        EltPos targetPos;
        memset(&targetPos, 0, sizeof(targetPos));
        for (int j = 0; j< AXIS_COUNT; j++)
        {
            originFile >> m_EltOriginPos[j];
        }
    }
    PrintJointData(m_EltOriginPos, "UpdateEltOrigin");
    ROS_INFO("[UpdateEltOrigin] end");
}

/*************************************************
Function: CEliteControl::EliteJointMove
Description: 艾利特机器人节点运动方式封装
Input: void
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteJointMove(elt_robot_pos &targetPos, string &sErr)
{
    ROS_INFO("[EliteMultiPointMove] start.");
    int ret;
    elt_error err;

    //6轴运动限位检测，防止报警
    for(int i=0; i<6 ;i++)
    {
        if((m_EliteCurrentPos[i] > AxisLimitAngle[i] && m_EliteCurrentPos[i] < targetPos[i])\
                    ||  (m_EliteCurrentPos[i] < AxisLimitAngle[i+6] && m_EliteCurrentPos[i] > targetPos[i]))
        {
            sErr.append("target pos error,the ").append(to_string(i+1)).append("th axis beyond the limit");
            ROS_ERROR("[EliteMultiPointMove] %s", sErr.c_str());
            return -1;
        }
    }

    ret = elt_joint_move( m_eltCtx, targetPos, m_dEltSpeed, &err);

    if (ret != ELT_SUCCESS)
    {
        if(err.err_msg[0] != '\0')
            sErr.append(err.err_msg);
        else
            sErr.append("elt joint move error");

        ROS_ERROR("[EliteMultiPointMove] err code: %d,err message: %s", err.code, err.err_msg);
        return -1;
    }

    ROS_INFO("[EliteMultiPointMove] after call joint move");
    return 1;
}

/*************************************************
Function: CEliteControl::EliteMultiPointMove
Description: 根据目标位置及当前位置插补出平滑轨迹点,并调用艾利特的轨迹运动函数
Input: void
Output: 1, 成功
       -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteMultiPointMove(elt_robot_pos &targetPos, string &sErrMsg)
{
    ROS_INFO("[EliteMultiPointMove] start");
    elt_error err;
    int ret;
    double dStepValue = 1;

    //6轴运动限位检测，防止报警
    for(int i=0; i<6 ;i++)
    {
        if((m_EliteCurrentPos[i] > AxisLimitAngle[i] && m_EliteCurrentPos[i] < targetPos[i])\
                    ||  (m_EliteCurrentPos[i] < AxisLimitAngle[i+6] && m_EliteCurrentPos[i] > targetPos[i]))
        {
            sErrMsg.append("target pos error,the ").append(to_string(i+1)).append("th axis beyond the limit");
            ROS_ERROR("[EliteMultiPointMove] %s", sErrMsg.c_str());
            return -1;
        }
    }

    //设置轨迹路点运动的速度
    ret = elt_set_waypoint_max_joint_speed( m_eltCtx, m_dEltSpeed, &err);

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
    while(ros::ok())
    {
        int nCount = 0;
        for(int i=0; i<ROBOT_POSE_SIZE; i++)
        {
            if(targetPos[i] - currentPos[i] >= 0)
            {
                targetPosTemp[i] += dStepValue;
                if(targetPosTemp[i] - targetPos[i] > 0.001)
                {
                    targetPosTemp[i] = targetPos[i];
                    nCount++;
                }
            }
            else
            {
                targetPosTemp[i] -= dStepValue;
                if(targetPosTemp[i] - targetPos[i] < 0.001)
                {
                    targetPosTemp[i] = targetPos[i];
                    nCount++;
                }
            }
        }

        ret = elt_add_waypoint( m_eltCtx, targetPosTemp, &err);
//        PrintJointData(targetPosTemp, "EliteMultiPointMove");

        if (ret != ELT_SUCCESS)
        {
            sErrMsg.append("elt add way point error");
            ROS_ERROR("[EliteMultiPointMove] add way point err code: %d,err message: %s", err.code, err.err_msg);
            return -1;
        }
        if(nCount == ROBOT_POSE_SIZE)
        {
            ROS_INFO("[EliteMultiPointMove] add way point done");
            break;
        }
    }

    //执行轨迹
    int nSmoothnessLevel = 7;
    int nMoveType = 0;
    ret = elt_track_move( m_eltCtx, nMoveType, nSmoothnessLevel, &err);

    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "elt track move error";
        ROS_INFO("[EliteMultiPointMove] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }
    ROS_INFO("[EliteMultiPointMove] call elt track move");
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
int CEliteControl::EliteStop(string sErr)
{
    int ret;
    elt_error err;
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

        ret = elt_set_servo_status(m_eltCtx, ENABLE, &err);
        if (ELT_SUCCESS != ret)
        {
            ROS_ERROR("[EliteDrag] enable elt servo failed. ret=%d, err.code=%d, err.msg=%s", ret, err.code, err.err_msg);
            return -1;
        }

        if(EliteSyncMotorStatus() == -1)
        {
            ROS_INFO("[EliteDrag] sync elite motor status failed");
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
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteRunDragTrack(const string &sFileName, double dSpeed, int nDirection, string &sErrMsg)
{
    ROS_INFO("[EliteRunDragTrack] start");
    elt_error err;
    int ret;
    ifstream trackFile;

    //打开轨迹文件
    trackFile.open(sFileName.c_str(), ios::out);
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
//    ret = elt_set_waypoint_max_rotate_speed( m_eltCtx, dSpeed,&err);
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
    deque<EltPos> trackDeque;
    while (!trackFile.eof())
    {
        EltPos targetPos;
        memset(&targetPos, 0, sizeof(targetPos));
        for (int j = 0; j< AXIS_COUNT; j++)
        {
            trackFile >> targetPos.eltPos[j];
        }
        trackDeque.push_back(targetPos);
    }
    trackDeque.pop_back();
    trackFile.close();

    //根据方向，调用elt库函数，将队列中的点添加的elt运动轨迹点当中
    while(!trackDeque.empty())
    {
        EltPos targetPosTemp;
        if(nDirection == FORWARD)
        {
            targetPosTemp= trackDeque.front();
            trackDeque.pop_front();
        }
        else if(nDirection == REVERSE)
        {
            targetPosTemp = trackDeque.back();
            trackDeque.pop_back();
        }

        ret = elt_add_waypoint( m_eltCtx, targetPosTemp.eltPos, &err);
//        PrintJointData(targetPosTemp.eltPos, "EliteRunDragTrack");
        if (ret != ELT_SUCCESS)
        {
            sErrMsg = "add way point err";
            ROS_INFO("[EliteRunDragTrack] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
            return -1;
        }
    }

    //执行轨迹
    int nSmoothnessLevel = 7;
    int nMoveType = 0;
    ret = elt_track_move( m_eltCtx, nMoveType, nSmoothnessLevel, &err);

    if (ret != ELT_SUCCESS)
    {
        sErrMsg = "elt track move error";
        ROS_INFO("[EliteRunDragTrack] %s,err code: %d,err message: %s",sErrMsg.c_str(), err.code, err.err_msg);
        return -1;
    }
    return 1;
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
bool CEliteControl::EliteGotoOrigin(const string &sInput, string &sOutput)
{
    ROS_INFO("[EliteGotoOrigin] start.");
    int nOrigin = 0;
    for(int i=0; i<6; i++)
    {
        if(abs(m_EliteCurrentPos[i] - m_EltOriginPos[i]) < 0.1)
        {
            nOrigin++;
        }
    }
    if(nOrigin == 6)
    {
        ROS_INFO("[EliteGotoOrigin] robot arm is already at origin.");
        return true;
    }

    if(sInput.empty() || sInput == "null")
    {
        if(EliteMultiPointMove(m_EltOriginPos, sOutput) == -1)
        {
            ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
            return false;
        }
    }
    else
    {
        if(EliteRunDragTrack(sInput, 40, REVERSE, sOutput) == -1)
        {
            ROS_ERROR("[EliteGotoOrigin]%s",sOutput.c_str());
            return false;
        }
    }
    return true;
}

/*************************************************
Function: CEliteControl::EliteSyncMotorStatus
Description: 同步机械臂数据
Input: void
Output:  1, 成功
        -1, 失败
Others: void
**************************************************/
int CEliteControl::EliteSyncMotorStatus()
{
    //获取同步状态,返回1，表示机器人处于同步状态。 返回0，表示机器人处于未同步状态。
    elt_error err;
    int nRet, nPowerStatus;

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

    if (ELT_SUCCESS == nRet)
    {
        if(ELT_FALSE  == nPowerStatus)
        {
            ROS_INFO("[EliteSyncMotorStatus] elt motor need sync.");

            //同步伺服编码器数据
            ROS_INFO("[EliteSyncMotorStatus] sync elt");
            nRet = elt_sync_motor_status(m_eltCtx, &err);

            if (ELT_SUCCESS != nRet)
            {
                ROS_INFO("[EliteSyncMotorStatus] sync motor failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
                return -1;
            }
        }
        ROS_INFO("[EliteSyncMotorStatus] motor status is synced");
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    else
    {
        ROS_INFO("[EliteSyncMotorStatus] get motor status failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
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

    ROS_INFO("[EliteClearAlarm] elt status is normal");
    return 1;
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
Function: CEliteControl::ArmServiceFunc
Description: service 功能函数
Input: wootion_msgs::ControlService::Request &Req, 请求
       wootion_msgs::ControlService::Response &Resp, 响应
Output: void
Others: void
**************************************************/
bool CEliteControl::ArmServiceFunc(wootion_msgs::ControlService::Request &Req, wootion_msgs::ControlService::Response &Resp)
{
    if (m_nCheckReceiver == 1 && Req.receiver != "robot_arm")
    {
        ROS_INFO("[ArmServiceFunc] receiver is:%s, ignore",Req.receiver.c_str());
        return true;
    }

    Resp.trans_id = Req.trans_id;
    Resp.ack = "failed";
    Resp.data.clear();

    ROS_INFO("[ArmServiceFunc] request:%s, data:%s", Req.cmd.c_str(), Req.data.c_str());

    if ((m_nEliteState == EMERGENCY_STOP || m_nEliteState == ALARM) && Req.cmd != "reset")
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

    if (m_nEliteState == MOVING && \
            (Req.cmd == "record_orbit" || Req.cmd == "play_orbit" || Req.cmd == "rotate" ||\
             Req.cmd == "set_orientation" || Req.cmd == "set_position" || Req.cmd == "reset"))
    {
        ROS_INFO("[ArmServiceFunc] robot arm is moving");
        Resp.data = "robot arm is moving";
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
    if (m_nCheckReceiver == 1 && ArmCmd->receiver != "cloud_terrace")
    {
        return;
    }

    wootion_msgs::GeneralAck ArmAck;

    ROS_INFO("[ArmCmdCallBack] cmd:%s, data:%s", ArmCmd->cmd.c_str(), ArmCmd->data.c_str());

    ArmAck.header.stamp = ros::Time::now();
    ArmAck.sender = "robot_arm";
    ArmAck.receiver = ArmCmd->sender;
    ArmAck.trans_id = ArmCmd->trans_id;
    ArmAck.ack = "failed";
    ArmAck.data.clear();

    if ((m_nEliteState == EMERGENCY_STOP || m_nEliteState == ALARM) && ArmCmd->cmd != "reset")
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

    if (m_nEliteState == MOVING && \
            (ArmCmd->cmd == "record_orbit" || ArmCmd->cmd == "play_orbit" || ArmCmd->cmd == "rotate" ||\
             ArmCmd->cmd == "set_orientation" || ArmCmd->cmd == "set_position" || ArmCmd->cmd == "reset"))
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
    m_sTaskName = sCommand.c_str();
    sOutput = "";

    if(m_bRecordDragTrack && sCommand != "record_orbit" && sCommand != "stop_orbit" && sCommand != "reset")
    {
        sOutput = "recording the arm orbit";
        return false;
    }

    if(sCommand == "record_orbit")
    {
        string sTrackFile = m_sArmTrackPath + m_sOrbitFileName;

        ROS_INFO("[ArmOperation] the orbit file is:%s",sTrackFile.c_str());
        if(m_TrackFile.OpenFile(sTrackFile,"w") == -1)
        {
            sOutput = "open file failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
        if(EliteDrag(ENABLE) == -1)
        {
            sOutput = "enable drag failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }

        sOutput = sTrackFile;
    }
    else if(sCommand == "stop_orbit")
    {
        if(EliteDrag(DISABLE) == -1)
        {
            sOutput = "disable drag failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
        if(m_TrackFile.CloseFile() == -1)
        {
            sOutput = "close file failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
        ROS_INFO("[ArmOperation] disable drag and close the orbit file");
    }
    else if(sCommand == "play_orbit")
    {
        string sTrackFile, sDirection;
        int nDirection;

        vector<string> vCmdList = SplitString(sInput, ",");

        if(vCmdList.size() == 1)
        {
            sDirection = vCmdList[0];
            nDirection = (sDirection == "+") ? FORWARD : REVERSE;
            sTrackFile = m_sArmTrackPath + m_sOrbitFileName;
        }
        else if(vCmdList.size() == 2)
        {
            sTrackFile = vCmdList[0];
            sDirection = vCmdList[1];
            nDirection = (sDirection == "+") ? FORWARD : REVERSE;
        }
        else
        {
            sOutput = "input error" + sInput;
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }

        if(EliteRunDragTrack(sTrackFile, 40, nDirection, sOutput) == -1)
        {
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
        if(vCmdList.size() == 2)
        {
            m_sResetOrbitFile = sTrackFile;
        }
    }
    else if(sCommand == "rotate")
    {
        std::string sYawAngle;
        std::string sPitchAngle;
        std::string sRollAngle;

        vector<string> vCmdList = SplitString(sInput, ",");

        if (vCmdList.size() < 3)
        {
            sOutput = "input error" + sInput;
            ROS_ERROR("[ArmOperation] %s",sOutput.c_str());
            return false;
        }

        sRollAngle = vCmdList[0];
        sPitchAngle = vCmdList[1];
        sYawAngle = vCmdList[2];

        elt_robot_pos targetPos;
        memcpy(targetPos, m_EliteCurrentPos, sizeof(targetPos));

        if(sRollAngle == "+")
        {
            targetPos[AXIS_ROLL] = AxisLimitAngle[AXIS_SIX_MAX];
            if(EliteJointMove(targetPos, sOutput) == -1)
            {
                sOutput.append(",roll + failed");
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
        }
        if(sRollAngle == "-")
        {
            targetPos[AXIS_ROLL] = AxisLimitAngle[AXIS_SIX_MIN];
            if(EliteJointMove(targetPos, sOutput) == -1)
            {
                sOutput.append(",roll - failed");
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
        }

        if(sPitchAngle == "+")
        {
            targetPos[AXIS_PITCH] = AxisLimitAngle[AXIS_FOUR_MAX];
            if(EliteJointMove(targetPos, sOutput) == -1)
            {
                sOutput.append(",tilt + failed");
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
        }
        if(sPitchAngle == "-")
        {
            targetPos[AXIS_PITCH] = AxisLimitAngle[AXIS_FOUR_MIN];
            if(EliteJointMove(targetPos, sOutput) == -1)
            {
                sOutput.append(",tilt - failed");
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
        }

        if(sYawAngle == "+")
        {
            targetPos[AXIS_YAW] = AxisLimitAngle[AXIS_FIVE_MAX];
            if(EliteJointMove(targetPos, sOutput) == -1)
            {
                sOutput.append(",pan + failed");
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
        }
        if(sYawAngle == "-")
        {
            targetPos[AXIS_YAW] = AxisLimitAngle[AXIS_FIVE_MIN];
            if(EliteJointMove(targetPos, sOutput) == -1)
            {
                sOutput.append(",pan - failed");
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
        }
    }
    else if(sCommand == "stop_rotate")
    {
        if(EliteStop(sOutput) == -1)
        {
            sOutput.append(",roll - failed");
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else if(sCommand == "get_orientation")
    {
        sOutput.append(to_string(m_EliteCurrentPos[AXIS_ROLL]));
        sOutput.append(",").append(to_string(m_EliteCurrentPos[AXIS_PITCH]));
        sOutput.append(",").append(to_string(m_EliteCurrentPos[AXIS_YAW]));
    }
    else if(sCommand == "set_orientation")
    {
        std::string sYawAngle;
        std::string sPitchAngle;
        std::string sRollAngle;

        vector<string> vCmdList = SplitString(sInput, ",");

        if (vCmdList.size() < 3)
        {
            sOutput = "input error" + sInput;
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }

        sRollAngle = vCmdList[0];
        sPitchAngle = vCmdList[1];
        sYawAngle = vCmdList[2];

        elt_robot_pos targetPos;
        memcpy(targetPos, m_EliteCurrentPos, sizeof(targetPos));

        targetPos[AXIS_ROLL] = strtod(sRollAngle.c_str(), nullptr);
        targetPos[AXIS_PITCH] = strtod(sPitchAngle.c_str(), nullptr);
        targetPos[AXIS_YAW] = strtod(sYawAngle.c_str(), nullptr);

        if(EliteMultiPointMove(targetPos, sOutput) == -1)
        {
            sOutput.append(",roll - failed");
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else if(sCommand == "move")
    {

    }
    else if(sCommand == "stop_move")
    {

    }
    else if(sCommand == "get_position")
    {
        elt_robot_pose response_pose_array;
        elt_error err;
        if(elt_positive_kinematic(m_eltCtx, m_EliteCurrentPos, response_pose_array, &err) == -1)
        {
            sOutput = "get position error:";
            sOutput.append(err.err_msg);
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }

        sOutput = to_string(response_pose_array[0]);
        sOutput.append(",").append(to_string(response_pose_array[1]));
        sOutput.append(",").append(to_string(response_pose_array[2]));
    }
    else if(sCommand == "set_position")
    {
        std::string sPosX;
        std::string sPosY;
        std::string sPosZ;

        vector<string> vCmdList = SplitString(sInput, ",");

        if (vCmdList.size() < 3)
        {
            sOutput = "input error" + sInput;
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
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
        if(elt_inverse_kinematic(m_eltCtx, targetPos, ElitePose, &err) == -1)
        {
            sOutput = "set position error:";
            sOutput.append(err.err_msg);
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }

        if(EliteMultiPointMove(targetPos, sOutput) == -1)
        {
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else if(sCommand == "reset")
    {
        if(m_nEliteState == ALARM)
        {
            elt_error err;
            int nRet = elt_clear_alarm(m_eltCtx, 1, &err);

            if (ELT_SUCCESS != nRet)
            {
                if(err.err_msg[0] != '\0')
                    sOutput.append(err.err_msg);
                else
                    sOutput.append("elt clear alarm error");

                ROS_ERROR("[ArmOperation] clear alarm failed. nRet=%d, err.code=%d, err.msg=%s", nRet, err.code, err.err_msg);
                return false;
            }
        }

        if(m_bRecordDragTrack)
        {
            if(EliteDrag(DISABLE) == -1)
            {
                sOutput = "disable drag failed";
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
            if(m_TrackFile.CloseFile() == -1)
            {
                sOutput = "close file failed";
                ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
                return false;
            }
            ROS_INFO("[ArmOperation] disable drag and close the orbit file");
        }

        if(!EliteGotoOrigin(m_sResetOrbitFile, sOutput))
        {
            return false;
        }
        m_sResetOrbitFile = "null";
    }
    else if (sCommand == "brush_on")
    {
        if(! m_ExtraCtrl.ExecuteTask(BURSH_ON))
        {
            sOutput = "brush on failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else if (sCommand == "brush_off")
    {
        if(! m_ExtraCtrl.ExecuteTask(BURSH_OFF))
        {
            sOutput = "brush off failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else if (sCommand == "light_on")
    {
        if(! m_ExtraCtrl.ExecuteTask(LIGHT_ON))
        {
            sOutput = "light on failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else if (sCommand == "light_off")
    {
        if(! m_ExtraCtrl.ExecuteTask(LIGHT_OFF))
        {
            sOutput = "light off failed";
            ROS_ERROR("[ArmOperation]%s",sOutput.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("unknown command:%s", sCommand.c_str());
        sOutput = "unknown command:" + sCommand;
        return false;
    }

    return true;
}
CEliteControl::~CEliteControl()
{
    UnInit();
    delete m_pControlThread;
    delete m_pEliteStatusThread;
}