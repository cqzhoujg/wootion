///*
// * preset.cpp
// *
// *  Created on: Aug 3, 2018
// *      Author: LaiHan
// *  Modified on: Aug 13, 2018
// *      Author: JiangXiaolu
// *  Modified on: Dec 27, 2018
// *      Author: LaiHan
// */

#include "preset_scale.h"

using namespace std;
using namespace cv;
using namespace ServiceCaller;

namespace PresetScale
{

vector<string> SplitString(const string &sInput, const string &sDelimiter)
{
    vector<string> vsResult;

    if (sInput.empty())
    {
        return vsResult;
    }

    char *szString = new char[sInput.length() + 1] ;
    strcpy(szString, sInput.c_str());

    char *szDelimiter = new char[sDelimiter.length() + 1];
    strcpy(szDelimiter, sDelimiter.c_str());

    char *pChar = strtok(szString, szDelimiter);

    while (pChar != nullptr)
    {
        string sSection = pChar;
        vsResult.emplace_back(sSection);
        pChar = strtok(nullptr, szDelimiter);
    }

    return vsResult;
}

PresetScale::PresetScale():
    m_PublicNodeHandle(),
    m_cNavigationStatus(0xff),
    m_bBusy(false),
    m_PrivateNodeHandle("~")
{
    m_PrivateNodeHandle.param("node_name", m_sNodeName, ros::this_node::getName());
    m_PrivateNodeHandle.param("preset_path", m_sPresetPath, string("preset"));
    m_PrivateNodeHandle.param("preset_file", m_sPresetFile, string("preset.ini"));
    m_PrivateNodeHandle.param("preset_scale_cmd_topic", m_sPresetScaleCmdTopic, string("preset_scale_cmd"));
    m_PrivateNodeHandle.param("preset_scale_ack_topic", m_sPresetScaleAckTopic, string("preset_scale_ack"));
    m_PrivateNodeHandle.param("nav_status_topic", m_sNavStatusTopic, string("nav_status"));
    m_PrivateNodeHandle.param("camera_service", m_sCameraService, string("camera_control"));
    m_PrivateNodeHandle.param("terrace_service", m_sTerraceService, string("terrace_control"));
    m_PrivateNodeHandle.param("infrared_service", m_sInfraredService, string("infrared_control"));
    m_PrivateNodeHandle.param("arm_service", m_sArmService, string("arm_control"));
    m_PrivateNodeHandle.param("preset_scale_service", m_sPresetService, string("preset_control"));
    m_PrivateNodeHandle.param("max_zoom_level", m_dMaxZoomLevel, 30.0);
    m_PrivateNodeHandle.param("min_zoom_level", m_dMinZoomLevel, 1.0);
    m_PrivateNodeHandle.param("middle_zoom_level", m_dMiddleZoomLevel, 12.0);
    m_PrivateNodeHandle.param("infrared_zoom_level", m_dInfraredZoomLevel, 2.0);
    m_PrivateNodeHandle.param("terrace_type", m_nTerraceType, 1);

    m_PresetCmdSub = m_PublicNodeHandle.subscribe(m_sPresetScaleCmdTopic, 10, &PresetScale::PresetCmdCallBack, this);
    m_PresetAckPub = m_PublicNodeHandle.advertise<wootion_msgs::PresetScaleAck>(m_sPresetScaleAckTopic, 10);
    m_NavStatusSub = m_PublicNodeHandle.subscribe(m_sNavStatusTopic, 10, &PresetScale::NavStatusCallBack, this);
    m_Service = m_PublicNodeHandle.advertiseService(m_sPresetService, &PresetScale::ServiceFunc, this);

    try
    {
        pCaller = new Caller({m_sCameraService, m_sTerraceService, m_sInfraredService, m_sArmService});
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("malloc service caller failed, %s", exception.what());
        exit(-1);
    }

    if (m_sPresetPath.front() == '/')
    {
        ROS_ERROR("input prest_path should not be leaded by \'/\', exiting");
        return;
    }

    if (m_sPresetPath.back() == '/')
    {
        ROS_WARN("input prest_path should not be ended by \'/\'");
        m_sPresetPath.pop_back();
    }

    ROS_INFO("build time:%s %s", __DATE__, __TIME__);
    ROS_INFO("node preset_scale started");
    ROS_INFO("node_name:%s", m_sNodeName.c_str());
    ROS_INFO("preset_path:%s", m_sPresetPath.c_str());
    ROS_INFO("preset_file:%s", m_sPresetFile.c_str());
    ROS_INFO("preset_scale_cmd_topic:%s", m_sPresetScaleCmdTopic.c_str());
    ROS_INFO("preset_scale_ack_topic:%s", m_sPresetScaleAckTopic.c_str());
    ROS_INFO("heartbeat_ack_topic:%s", m_sNavStatusTopic.c_str());
    ROS_INFO("camera_service:%s", m_sCameraService.c_str());
    ROS_INFO("terrace_service:%s", m_sTerraceService.c_str());
    ROS_INFO("infrared_service:%s", m_sInfraredService.c_str());
    ROS_INFO("arm_service:%s", m_sArmService.c_str());
    ROS_INFO("preset_scale_service:%s", m_sPresetService.c_str());
    ROS_INFO("max_zoom_level:%.1f", m_dMaxZoomLevel);
    ROS_INFO("min_zoom_level:%.1f", m_dMinZoomLevel);
    ROS_INFO("middle_zoom_level:%.1f", m_dMiddleZoomLevel);
    ROS_INFO("infrared_zoom_level:%.1f", m_dInfraredZoomLevel);
    ROS_INFO("terrace_type:%d", m_nTerraceType);
}

PresetScale::~PresetScale()
{
    //nothing to do
}

void PresetScale::AdjustYawAngle(int &nAngle)
{
    while (nAngle <= -18000 || nAngle > 18000)
    {
        if (nAngle > 18000)
        {
            nAngle -= 36000;
        }
        else
        {
            nAngle += 36000;
        }
    }
}

//create multiple level folder
bool PresetScale::CreateDir(const char *sPathName)
{
    char DirName[256] = {0};
    unsigned long i;
    unsigned long len;

    strncpy(DirName, sPathName, 256);
    len = strlen(DirName);
    if (DirName[len - 1] != '/')
    {
        strcat(DirName, "/");
        len += 1;
    }

    for (i = 1; i < len; i++)
    {
        if (DirName[i] == '/')
        {
            DirName[i] = 0;
            if (access(DirName, F_OK) != 0)
            {
                if (mkdir(DirName, 0755) == -1)
                {
                    perror("mkdir error");
                    ROS_ERROR("create directory failed, errno:%d", errno);
                    return false;
                }
            }
            DirName[i] = '/';
        }
    }

    return true;
}

string PresetScale::GetTimeString()
{
    char szPresentTime[50] = {0};
    string sTime;
    time_t now;
    struct tm *tm_now;

    now = time(nullptr);
    tm_now = localtime(&now);
    strftime(szPresentTime, 50, "%Y%m%d_%H%M%S", tm_now);
    sTime = szPresentTime;

    return sTime;
}

bool PresetScale::SaveMainIni()
{
    double dRobotPositionX;
    double dRobotPositionY;

    if (!write_profile_int("main", "infrared", m_nInfrared, m_sIniFile.c_str()))
    {
        ROS_ERROR("save infrared failed");
        return false;
    }

    if (!write_profile_string("main", "type", m_sType.c_str(), m_sIniFile.c_str()))
    {
        ROS_ERROR("save type failed");
        return false;
    }

    if (!write_profile_string("main", "preset_id", m_sPresetId.c_str(), m_sIniFile.c_str()))
    {
        ROS_ERROR("save type failed");
        return false;
    }

    if (!write_profile_string("main", "record_time", m_sRecordTime.c_str(), m_sIniFile.c_str()))
    {
        ROS_ERROR("save type failed");
        return false;
    }

    if (!write_profile_int("main", "levels", m_nLevelCount, m_sIniFile.c_str()))
    {
        ROS_ERROR("save record levels failed");
        return false;
    }

    m_RobotStatusMutex.lock();
    dRobotPositionX = m_dRobotPositionX;
    dRobotPositionY = m_dRobotPositionY;
    m_RobotStatusMutex.unlock();

    if (!write_profile_double("main", "robot_position_x", dRobotPositionX, m_sIniFile.c_str()))
    {
        ROS_ERROR("save robot position x failed");
        return false;
    }

    if (!write_profile_double("main", "robot_position_y", dRobotPositionY, m_sIniFile.c_str()))
    {
        ROS_ERROR("save robot position y failed");
        return false;
    }

#if 0
    //do not write into ini file, manual add thess configures if neccessary
    if (!write_profile_int("main", "end_level", 0, m_sIniFile.c_str()))
    {
        ROS_ERROR("save end level failed");
        return false;
    }

    if (!write_profile_int("main", "wait_time", 0, m_sIniFile.c_str()))
    {
        ROS_ERROR("save wait time failed");
        return false;
    }
#endif

    return true;
}

bool PresetScale::SaveTerraceIni()
{
    string sOutput;
    char *pEnd = nullptr;
    int nRobotYawAngle;
    int nTerracePanAngle;
    int nTiltAngle;
    int nPanAngle;

    if (!pCaller->Call(get_pan_angle, "", sOutput))
    {
        ROS_ERROR("get terrace pan angle failed, output:%s", sOutput.c_str());
        return false;
    }

    nTerracePanAngle = (int)strtol(sOutput.c_str(), &pEnd, 10);
    m_RobotStatusMutex.lock();
    nRobotYawAngle = (int)round(m_dRobotOrientation * 18000.0 / M_PI);
    m_RobotStatusMutex.unlock();
    nPanAngle = nRobotYawAngle + nTerracePanAngle;
    AdjustYawAngle(nPanAngle);

    if (!write_profile_int("terrace", "yaw_angle", nPanAngle, m_sIniFile.c_str()))
    {
        ROS_ERROR("save pan angle failed");
        return false;
    }

    if (!pCaller->Call(get_tilt_angle, "", sOutput))
    {
        ROS_ERROR("get terrace tilt angle failed, output:%s", sOutput.c_str());
        return false;
    }

    nTiltAngle = (int)strtol(sOutput.c_str(), &pEnd, 10);

    if (!write_profile_int("terrace", "pitch_angle", nTiltAngle, m_sIniFile.c_str()))
    {
        ROS_ERROR("save tilt angle failed");
        return false;
    }

    return true;
}

bool PresetScale::SaveArmIni()
{
    string sOutput;
    string sArmOrbit;

    if(m_sExtraData == "1")
    {
        //轨迹文件名写入INI
        sArmOrbit = m_sDirectory + sArmOrbitFile;
        if (!write_profile_string("arm", "arm_file", sArmOrbit.c_str(), m_sIniFile.c_str()))
        {
            ROS_ERROR("save arm file path failed");
            return false;
        }
        //轨迹文件拷贝
        //判断轨迹源文件是否存在
        if(access(sArmOrbitFixedPathName.c_str(),0) != 0)
        {
            ROS_ERROR("orbit file is null");
            return false;
        }
        string command = "cp ";
        command  += sArmOrbitFixedPathName;
        command  += " ";
        command  += m_sDirectory;
        system((char*)command.c_str());

        //判断轨迹源文件是否拷贝成功
        if(access((m_sDirectory+sArmOrbitFile).c_str(),0) != 0)
        {
            ROS_ERROR("copy file failed");
            return false;
        }
    }
    else
    {
        sArmOrbit = "null";
        if (!write_profile_string("arm", "arm_file", sArmOrbit.c_str(), m_sIniFile.c_str()))
        {
            ROS_ERROR("save arm file path failed");
            return false;
        }
    }

//    if (!pCaller->Call(get_position, "", sOutput))
//    {
//        ROS_ERROR("get arm position failed, output:%s", sOutput.c_str());
//        return false;
//    }
//
//    if (!write_profile_string("arm", "position", sOutput.c_str(), m_sIniFile.c_str()))
//    {
//        ROS_ERROR("save arm position failed");
//        return false;
//    }

    if (!pCaller->Call(get_orientation, "", sOutput))
    {
        ROS_ERROR("get arm orientation failed, output:%s", sOutput.c_str());
        return false;
    }

    if (!write_profile_string("arm", "orientation", sOutput.c_str(), m_sIniFile.c_str()))
    {
        ROS_ERROR("save arm orientation failed");
        return false;
    }

    return true;
}

bool PresetScale::SaveCameraIni(int nLevel, double dZoomLevel)
{
    string sOutput;
    string sLevel;
    string sPictureFile;

    sLevel = "zoom" + to_string(nLevel);
    sPictureFile = m_sDirectory + sLevel + ".jpg";

    ROS_INFO("SaveCameraIni, level:%d, zoom:%.1f", nLevel, dZoomLevel);

    if (!pCaller->Call(capture_picture, sPictureFile, sOutput))
    {
        ROS_ERROR("save picture file:%s failed, output:%s", sPictureFile.c_str(), sOutput.c_str());
        return false;
    }

    if (!write_profile_double(sLevel.c_str(), "zoom_level", dZoomLevel, m_sIniFile.c_str()))
    {
        ROS_ERROR("save zoom level failed");
        return false;
    }

    if (!pCaller->Call(get_wdr_level, "", sOutput))
    {
        ROS_ERROR("get wdr level failed, output:%s", sOutput.c_str());
        return false;
    }
    if (!write_profile_int(sLevel.c_str(), "wdr_level", stoi(sOutput), m_sIniFile.c_str()))
    {
        ROS_ERROR("save wdr level failed");
        return false;
    }

    if (!pCaller->Call(get_min_focus_distance, "", sOutput))
    {
        ROS_ERROR("get min focus distance failed, output:%s", sOutput.c_str());
        return false;
    }
    if (!write_profile_int(sLevel.c_str(), "min_focus_distance", stoi(sOutput), m_sIniFile.c_str()))
    {
        ROS_ERROR("save min focus distance failed");
        return false;
    }

    if (!pCaller->Call(get_focus_position, "", sOutput))
    {
        ROS_ERROR("get focus position failed, output:%s", sOutput.c_str());
        return false;
    }
    if (!write_profile_int(sLevel.c_str(), "focus_position", stoi(sOutput), m_sIniFile.c_str()))
    {
        ROS_ERROR("save focus position failed");
        return false;
    }

    return true;
}

void PresetScale::PublishPresetScaleAck(wootion_msgs::PresetScaleAck &PresetAck)
{
    PresetAck.header.stamp = ros::Time::now();
    PresetAck.sender = m_sNodeName;
    m_PresetAckPub.publish(PresetAck);

    ROS_INFO("<==%s, trans_id:%d, levels:%d, result:%s, directory:%s",
             m_sPresetScaleAckTopic.c_str(), PresetAck.trans_id, m_nLevelCount, PresetAck.result.c_str(), PresetAck.directory.c_str());
}

void PresetScale::PresetCmdCallBack(const wootion_msgs::PresetScaleCmd::ConstPtr &PresetCmd)
{
    wootion_msgs::PresetScaleAck Ack;
    string sResult = "failed";

    ROS_INFO("==>%s, trans_id:%u, preset_id:%s, infrared:%d, type:%s",
             m_sPresetScaleCmdTopic.c_str(), PresetCmd->trans_id, PresetCmd->preset_id.c_str(), PresetCmd->infrared, PresetCmd->type.c_str());

    m_sPresetId = PresetCmd->preset_id;
    m_nInfrared = PresetCmd->infrared;
    m_sType = PresetCmd->type;
    m_sExtraData = PresetCmd->extra_data;

    PresetScaleProc(sResult);

    Ack.receiver = PresetCmd->sender;
    Ack.trans_id = PresetCmd->trans_id;
    Ack.result = sResult;
    Ack.directory = m_sDirectory;

    PublishPresetScaleAck(Ack);
}


bool PresetScale::ServiceFunc(wootion_msgs::ControlService::Request &Req, wootion_msgs::ControlService::Response &Resp)
{
    vector<string> vsData;
    string sResult = "failed";

    ROS_INFO("==>%s, trans_id:%u, cmd:%s, data:%s",
             m_sPresetService.c_str(), Req.trans_id, Req.cmd.c_str(), Req.data.c_str());

    Resp.trans_id = Req.trans_id;
    Resp.ack = sResult;
    Resp.data.clear();

    vsData = SplitString(Req.data, ",");
    if (vsData.size() < 4)
    {
        ROS_ERROR("input:%s error", Req.data.c_str());
        return false;
    }

    m_sPresetId = vsData[0];
    m_nInfrared = std::stoi(vsData[1]);
    m_sType = vsData[2];
    m_sExtraData = vsData[3];

    PresetScaleProc(sResult);

    Resp.ack = sResult;
    Resp.data = m_sDirectory;

    ROS_INFO("<==%s, trans_id:%d, levels:%d, ack:%s, data:%s",
             m_sPresetService.c_str(), Resp.trans_id, m_nLevelCount, Resp.ack.c_str(), Resp.data.c_str());

    return true;
}

void PresetScale::PresetScaleProc(std::string &sResult)
{
    vector<double> vdPresetLevels;
    vector<double>::iterator it;
    string sOutput;

    if (m_bBusy)
    {
        ROS_ERROR("preset_scale node is busy, failed");
        return;
    }

    m_RobotStatusMutex.lock();
    if ((m_cNavigationStatus & cNavWarnFlag) == cNavWarnFlag)
    {
        ROS_ERROR("nav_status:%d abnormal, failed", m_cNavigationStatus);
        m_RobotStatusMutex.unlock();
        return;
    }
    m_RobotStatusMutex.unlock();

    m_bBusy = true;

    m_sRecordTime = GetTimeString();
    m_sDirectory = string(getenv("HOME")) + "/" + m_sPresetPath + "/" + m_sPresetId + "/" + m_sRecordTime + "/";
    m_sIniFile = m_sDirectory + m_sPresetFile;
    m_nLevelCount = 1;

    if (!CreateDir(m_sDirectory.c_str()))
    {
        ROS_ERROR("create directory:%s failed", m_sDirectory.c_str());
        m_bBusy = false;
        return;
    }

    if (!pCaller->Call(set_focus_mode, "auto", sOutput))
    {
        ROS_ERROR("set focus mode to auto failed, output:%s", sOutput.c_str());
        m_bBusy = false;
        return;
    }

    if (m_nTerraceType == 0)
    {
        if (!SaveTerraceIni())
        {
            ROS_ERROR("save terrace info failed");
            m_bBusy = false;
            return;
        }
    }
    else if (m_nTerraceType == 1)
    {
        if (!SaveArmIni())
        {
            ROS_ERROR("save arm info failed");
            m_bBusy = false;
            return;
        }
    }

    if (m_nInfrared == 1)
    {
        m_dPresetZoomLevel = m_dInfraredZoomLevel;
        if (!pCaller->Call(set_zoom_level, to_string(m_dInfraredZoomLevel), sOutput))
        {
            ROS_ERROR("set zoom level:%.1f for infrared failed, output:%s", m_dInfraredZoomLevel, sOutput.c_str());
            m_bBusy = false;
            return;
        }

        if (!SaveCameraIni(0, m_dInfraredZoomLevel))
        {
            ROS_ERROR("save camera info for infrared failed");
            m_bBusy = false;
            return;
        }

        if (!pCaller->Call(save_temperature, m_sDirectory + "zoom0_infrared", sOutput))
        {
            ROS_ERROR("infrared capture picture:%s failed, output:%s", (m_sDirectory + "zoom0_infrared").c_str(), sOutput.c_str());
            m_bBusy = false;
            return;
        }
    }
    else
    {
        if (!pCaller->Call(get_zoom_level, "", sOutput))
        {
            ROS_ERROR("get zoom level failed, output:%s", sOutput.c_str());
            m_bBusy = false;
            return;
        }
        m_dPresetZoomLevel = (double)stof(sOutput);

        //save zoom0 for preset
        if (!SaveCameraIni(0, m_dPresetZoomLevel))
        {
            ROS_ERROR("save camera info for level 0 failed");
            m_bBusy = false;
            return;
        }

        if (m_dMinZoomLevel >= m_dPresetZoomLevel)
        {
            //save 1 level
            //zoom0:PresetZoomLevel
            vdPresetLevels.clear();
        }
        else
        {
            if (m_dPresetZoomLevel <= m_dMiddleZoomLevel || m_dMinZoomLevel >= m_dMiddleZoomLevel)
            {
                //save 2 levels
                //zoom0:PresetZoomLevel
                //zoom1:MinZoomLevel
                vdPresetLevels.push_back(m_dMinZoomLevel);
            }
            else //m_dMinZoomLevel < m_dMiddleZoomLevel < m_dPresetZoomLevel
            {
                //save 3 levels
                //zoom0:PresetZoomLevel
                //zoom1:round(sqrt(m_dPresetZoomLevel * m_dMinZoomLevel) * 10) / 10
                //zoom2:MinZoomLevel
                vdPresetLevels.push_back(round(sqrt(m_dPresetZoomLevel * m_dMinZoomLevel) * 10) / 10);
                vdPresetLevels.push_back(m_dMinZoomLevel);
            }
        }

        for (it = vdPresetLevels.begin(); it != vdPresetLevels.end(); ++it)
        {
            if (!pCaller->Call(set_zoom_level, to_string(*it), sOutput))
            {
                ROS_ERROR("set zoom level:%.1f failed, output:%s", *it, sOutput.c_str());
                m_bBusy = false;
                return;
            }

            if (!SaveCameraIni(m_nLevelCount, *it))
            {
                ROS_ERROR("save camera info for level:%d, zoom:%.1f failed", m_nLevelCount, *it);
                m_bBusy = false;
                return;
            }

            m_nLevelCount++;
        }
    }

    if (!SaveMainIni())
    {
        ROS_ERROR("save main info failed");
        m_bBusy = false;
        return;
    }

    if (!pCaller->Call(set_zoom_level, to_string(m_dPresetZoomLevel), sOutput))
    {
        ROS_ERROR("pre set zoom level:%.1f failed, output:%s", m_dPresetZoomLevel, sOutput.c_str());
    }

    sResult = "success";
    m_bBusy = false;
}

void PresetScale::NavStatusCallBack(const wootion_msgs::NavStatus_<std::allocator<void>>::ConstPtr &NavStatus)
{
    m_RobotStatusMutex.lock();
    m_dRobotPositionX = NavStatus->position[0];
    m_dRobotPositionY = NavStatus->position[1];
    m_dRobotOrientation = NavStatus->orientation;
    m_cNavigationStatus = NavStatus->nav_status;
    m_RobotStatusMutex.unlock();
}

}
