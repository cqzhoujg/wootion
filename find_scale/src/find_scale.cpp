#include "find_scale.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace ServiceCaller;

namespace FindScale
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

FindScale::FindScale():
    m_bExecute(false),
    m_cNavigationStatus(0xff),
    m_sSender(""),
    m_PrivateNodeHandle("~"),
    m_PublicNodeHandle(""),
    m_dPreZoomLevel(30.0),
    m_nLastRotatePanAngle(36000),
    m_nLastRotateTiltAngle(18000),
    m_nLastPresetYawAngle(0),
    m_nLastPresetPitchAngle(0),
    m_dLastResult(0.0),
    m_eWorkStatus(idle),
    m_ActionlibServer(m_PublicNodeHandle, "find_scale_action", boost::bind(&FindScale::ExecuteActionCb, this, _1), false),
    m_bActionLive(false)
{
    m_PrivateNodeHandle.param("node_name", m_sNodeName, ros::this_node::getName());
    m_PrivateNodeHandle.param("preset_file", m_sPresetFile, std::string(("preset.ini")));
    m_PublicNodeHandle.param("camera_service", m_sCameraService, std::string("camera_control"));
    m_PublicNodeHandle.param("terrace_service", m_sTerraceService, std::string("terrace_control"));
    m_PublicNodeHandle.param("infrared_service", m_sInfraredService, std::string("infrared_control"));
    m_PrivateNodeHandle.param("arm_service", m_sArmService, string("arm_control"));
    m_PublicNodeHandle.param("find_scale_cmd_topic", m_sFindScaleCmdTopic, std::string("find_scale_cmd"));
    m_PublicNodeHandle.param("find_scale_ack_topic", m_sFindScaleAckTopic, std::string("find_scale_ack"));
    m_PublicNodeHandle.param("nav_status_topic", m_sNavStatusTopic, std::string("nav_status"));
    m_PrivateNodeHandle.param("min_rotate_pan_angle", m_nMinRotatePanAngle, 10);
    m_PrivateNodeHandle.param("min_rotate_tilt_angle", m_nMinRotateTiltAngle, 10);
    m_PrivateNodeHandle.param("min_delta_rotate_angle", m_nMinDeltaRotateAngle, 10);
    m_PrivateNodeHandle.param("level_find_time", m_nLevelFindTimeout, 20);
    m_PrivateNodeHandle.param("max_find_time", m_nMaxFindTimeout, 60);
    m_PrivateNodeHandle.param("find_wait_time", m_nFindWaitTime, 0);
    m_PrivateNodeHandle.param("level0_step", m_nMaxStep[0], 3);
    m_PrivateNodeHandle.param("level1_step", m_nMaxStep[1], 2);
    m_PrivateNodeHandle.param("level2_step", m_nMaxStep[2], 2);
    m_PrivateNodeHandle.param("skip_level", m_bSkipLevel, false);
    m_PrivateNodeHandle.param("max_offset_delta", m_dMaxOffsetDelta, 0.5);
    m_PrivateNodeHandle.param("environment_type", m_nEnvironmentType, 0);
    m_PrivateNodeHandle.param("one_shot_max_rotate_angle", m_nOneShotMaxRotateAngle, 1000);
    m_PrivateNodeHandle.param("max_rotate_angle_delta", m_nMaxRotateAngleDelta, 100);
    m_PrivateNodeHandle.param("same_position_delta", m_dSamePositionDelta, 0.01);
    m_PrivateNodeHandle.param("same_orientation_delta", m_dSameOrientationDelta, 0.01);
    m_PrivateNodeHandle.param("similar_angle_delta", m_nSimilarAngleDelta, 2000);
    m_PrivateNodeHandle.param("save_infrared_picture", m_nSaveInfraredPicture, 0);
    m_PrivateNodeHandle.param("terrace_type", m_nTerraceType, 1);
    if(1 == m_nTerraceType)
    {
        m_PublicNodeHandle.param("arm_heart_beat_topic", m_sArmHeartBeatTopic, std::string("arm_heart_beat"));
    }

    ROS_INFO("build time:%s %s", __DATE__, __TIME__);
    ROS_INFO("node_name:%s", m_sNodeName.c_str());
    ROS_INFO("preset_file:%s", m_sPresetFile.c_str());
    ROS_INFO("camera_service:%s", m_sCameraService.c_str());
    ROS_INFO("terrace_service:%s", m_sTerraceService.c_str());
    ROS_INFO("infrared_service:%s", m_sInfraredService.c_str());
    ROS_INFO("arm_service:%s", m_sArmService.c_str());
    ROS_INFO("find_scale_cmd_topic:%s", m_sFindScaleCmdTopic.c_str());
    ROS_INFO("find_scale_ack_topic:%s", m_sFindScaleAckTopic.c_str());
    ROS_INFO("nav_status_topic_topic:%s", m_sNavStatusTopic.c_str());
    ROS_INFO("min_rotate_pan_angle:%d", m_nMinRotatePanAngle);
    ROS_INFO("min_rotate_tilt_angle:%d", m_nMinRotateTiltAngle);
    ROS_INFO("min_delta_rotate_angle:%d", m_nMinDeltaRotateAngle);
    ROS_INFO("level_find_time:%d", m_nLevelFindTimeout);
    ROS_INFO("max_find_time:%d", m_nMaxFindTimeout);
    ROS_INFO("find_wait_time:%d", m_nFindWaitTime);
    ROS_INFO("level0_step:%d", m_nMaxStep[0]);
    ROS_INFO("level1_step:%d", m_nMaxStep[1]);
    ROS_INFO("level2_step:%d", m_nMaxStep[2]);
    ROS_INFO("skip_level:%d", m_bSkipLevel);
    ROS_INFO("max_offset_delta:%.2f", m_dMaxOffsetDelta);
    ROS_INFO("environment_type:%d", m_nEnvironmentType);
    ROS_INFO("one_shot_max_rotate_angle:%d", m_nOneShotMaxRotateAngle);
    ROS_INFO("max_rotate_angle_delta:%d", m_nMaxRotateAngleDelta);
    ROS_INFO("same_position_delta:%.2f", m_dSamePositionDelta);
    ROS_INFO("same_orientation_delta:%.2f", m_dSameOrientationDelta);
    ROS_INFO("similar_angle_delta:%d", m_nSimilarAngleDelta);
    ROS_INFO("save_infrared_picture:%d", m_nSaveInfraredPicture);
    ROS_INFO("terrace_type:%d", m_nTerraceType);

    m_ActionlibServer.registerPreemptCallback(boost::bind(&FindScale::PreemptCb, this));
    m_ActionlibServer.start();
    m_NavStatusSub = m_PublicNodeHandle.subscribe<wootion_msgs::NavStatus>(m_sNavStatusTopic, 1, &FindScale::NavStatusCB, this);

    try
    {
        m_pFindScaleThread = new std::thread(std::bind(&FindScale::FindScaleThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("malloc find scale thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        pCaller = new Caller({m_sCameraService, m_sTerraceService, m_sInfraredService, m_sArmService});
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("malloc service caller failed, %s", exception.what());
        exit(-1);
    }

}

FindScale::~FindScale()
{
    m_pFindScaleThread->join();
    delete m_pFindScaleThread;
}

void FindScale::ExecuteActionCb(const wootion_msgs::GeneralGoalConstPtr &goal)
{
    using namespace boost::property_tree;
    std::string sOutput;
    wootion_msgs::GeneralFeedback FeedBack;
    wootion_msgs::GeneralResult Result;
    std::stringstream ss(goal->data);
    std::string sPresetPath;
    ptree pt;

    Result.action_id = goal->action_id;
    m_sTransId = goal->action_id;
    m_nForeignDetect = 0;
    m_sGoalType = goal->type;
    m_bActionLive = true;

    try
    {
        read_json(ss, pt);
        sPresetPath = pt.get<std::string>("preset_path");
    }
    catch (ptree_error &e)
    {
        ROS_ERROR("parse json error");
        ResultFailed("parse json error");
        return;
    }

    if (sPresetPath.empty())
    {
        ROS_ERROR("invalid preset path");
        ResultFailed("invalid preset path");
        return;
    }

    ROS_INFO("==>find_scale_action, trans_id:%s, goal type:%s, preset path:%s",
             goal->action_id.c_str(), goal->type.c_str(), sPresetPath.c_str());

    if (goal->type == "pre_find" && m_nTerraceType == 0)
    {
        if (PreFind(sPresetPath))
        {
            SetWorkStatus(idle);
            Result.ret_code = "0";
            if (m_ActionlibServer.isActive())
            {
                m_ActionlibServer.setSucceeded(Result);
            }
        }
        else
        {
            ResultFailed("pre_find error");
        }
        return;
    }
    else if (goal->type == "find_scale")
    {
        m_nForeignDetect = 0;
    }
    else if (goal->type == "find_foreign")
    {
        m_nForeignDetect = 1;
    }
    else if (goal->type == "find_both")
    {
        m_nForeignDetect = 1;
    }
    else
    {
        ResultFailed("Unknown action type");
        return;
    }

    FindScaleCmdCB(sPresetPath);

    while (m_bActionLive)
    {
        //nothing to do
    }
}

void FindScale::PreemptCb()
{
    if (m_ActionlibServer.isActive())
    {
        m_ActionlibServer.setPreempted();
    }
}

bool FindScale::PreFind(const std::string &sPresetPath)
{
    double dRobotPositionX;
    double dRobotPositionY;
    m_sPresetPath = sPresetPath;
    if (m_sPresetPath.back() != '/')
    {
        m_sPresetPath.push_back('/');
    }
    m_sIniFile = m_sPresetPath + sPresetConfigFile;
    if (!read_profile_double("main", "robot_position_x", &dRobotPositionX, m_sIniFile.c_str()))
    {
        ROS_ERROR("read robot position x failed");
        return false;
    }

    if (!read_profile_double("main", "robot_position_y", &dRobotPositionY, m_sIniFile.c_str()))
    {
        ROS_ERROR("read robot position y failed");
        return false;
    }

    m_dPreRobotOrientation = atan2(dRobotPositionY - m_dRobotPositionY, dRobotPositionX - m_dRobotPositionX);

    if (!SetTerracePreset(true))
    {
        ROS_WARN("pre rotate cloud terrace failed");
    }

    if (!read_profile_int("main", "levels", &m_nPresetLevels, m_sIniFile.c_str()))
    {
        ROS_ERROR("read levels failed");
        return false;
    }

    if (!SetCameraPreset(m_nPresetLevels - 1))
    {
        ROS_WARN("pre set camera for level:%d failed", m_nPresetLevels - 1);
    }

    return true;
}

void FindScale::AdjustYawAngle(int &nAngle)
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

void FindScale::AdjustPitchAngle(int &nAngle)
{
    while (nAngle < -9000 || nAngle > 9000)
    {
        if (nAngle > 9000)
        {
            nAngle -= 36000;
        }
        else
        {
            nAngle += 36000;
        }
    }
}

void FindScale::FindScaleThreadFunc()
{
    wootion_msgs::GeneralFeedback FeedBack;
    bool bResult;

    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_Mutex);
        if (!m_bExecute)
        {
            m_Condition.wait(lock);
        }
        m_bExecute = false;
        lock.unlock();

        FeedBack.action_id = m_sTransId;
        FeedBack.event_code = "1";
        FeedBack.event_msg = "failed";
        FeedBack.data = "failed";
        switch (GetWorkStatus())
        {
            case stop:
                ResultFailed("work status is stop");
                break;
            case work:
                bResult = FindTarget();
                if (GetWorkStatus() == stop)
                {
                    ResultFailed("work status is stop");
                    break;
                }
                if (bResult)
                {
                    ResultSucceed();
                }
                else
                {
                    ResultFailed("FindTarget Failed");
                }
                break;
            default:
                ROS_ERROR("unknown work status");
                ResultFailed("unknown work status");
                break;
        }
        m_bActionLive = false;
    }
}

void FindScale::ResultSucceed()
{
    SetWorkStatus(idle);
    wootion_msgs::GeneralResult Result;
    boost::property_tree::ptree pt_allitem;
    boost::property_tree::ptree pt_item1, pt_item2, pt_item3, pt_item4;
    boost::property_tree::ptree children, child;
    std::vector<int>::iterator it;

    Result.action_id = m_sTransId;
    Result.ret_code = "0";

    pt_allitem.put("filename", m_sPictureFileName);
    pt_allitem.put("infrared_filename", m_sInfraredFileName);

    for (const auto &val : m_vsForeignPictureFileName)
    {
        children.clear();
        child.clear();
        child.put("", val);
        pt_item1.push_back(std::make_pair("", child));
    }
    pt_allitem.add_child("sForeignFilename", pt_item1);

    for (it = m_vnRoiVertex.begin(); it != m_vnRoiVertex.end(); ++it)
    {
        children.clear();
        child.clear();
        child.put<int>("", *it);
        children.push_back(std::make_pair("", child));
        ++it;
        child.put<int>("", *it);
        children.push_back(std::make_pair("", child));
        pt_item2.push_back(std::make_pair("", children));
    }
    pt_allitem.add_child("roi_vertex", pt_item2);

    for (it = m_vnRoiVertexInfrared.begin(); it != m_vnRoiVertexInfrared.end(); ++it)
    {
        children.clear();
        child.clear();
        child.put<int>("", *it);
        children.push_back(std::make_pair("", child));
        ++it;
        child.put<int>("", *it);
        children.push_back(std::make_pair("", child));
        pt_item3.push_back(std::make_pair("", children));
    }
    pt_allitem.add_child("infrared_roi_vertex", pt_item3);

    std::stringstream ss;
    boost::property_tree::write_json(ss, pt_allitem);
    Result.data = ss.str();
    if (m_ActionlibServer.isActive())
    {
        m_ActionlibServer.setSucceeded(Result);
    }

    std::string sOutput;
    m_dLastRobotPositionX = m_dRobotPositionX;
    m_dLastRobotPositionY = m_dRobotPositionY;
    m_dLastRobotOrientation = m_dRobotOrientation;
    m_nLastRotatePanAngle = m_nRotatePanAngle;
    m_nLastRotateTiltAngle = m_nRotateTiltAngle;
    m_nLastPresetYawAngle = m_nPresetYawAngle;
    m_nLastPresetPitchAngle = m_nPresetPitchAngle;

    if (m_nEnvironmentType == 0)
    {
        if (!CallControl(set_zoom_level, std::to_string(m_dPreZoomLevel), sOutput))
        {
            ROS_ERROR("pre set zoom level :%.1f failed, output:%s", m_dPreZoomLevel, sOutput.c_str());
        }
    }

    ROS_INFO("<==find_scale_action success, trans_id:%s, data:%s", m_sTransId.c_str(), Result.data.c_str());
}

void FindScale::ResultFailed(const std::string &sReason)
{
    SetWorkStatus(idle);
    wootion_msgs::GeneralResult Result;
    boost::property_tree::ptree pt_allitem;

    pt_allitem.put("sReason", sReason);
    std::stringstream ss;
    boost::property_tree::write_json(ss, pt_allitem);
    Result.data = ss.str();
    Result.ret_code = "1";
    Result.action_id = m_sTransId;

    if (m_ActionlibServer.isActive())
    {
        m_ActionlibServer.setSucceeded(Result);
    }

    ROS_INFO("<==find_scale_action failed, trans_id:%s, data:%s", m_sTransId.c_str(), Result.data.c_str());
}

void FindScale::FindScaleCmdCB(const std::string &sPresetPath)
{
    double dRobotPositionX;
    double dRobotPositionY;

    std::this_thread::sleep_for(std::chrono::seconds(m_nFindWaitTime));

    if (GetWorkStatus() != idle)
    {
        ROS_ERROR("==>find_scale_cmd, action_id:%s, but work status is not idle, failed", m_sTransId.c_str());
        ResultFailed("work status is not idle");
        return;
    }
    m_sPresetPath = sPresetPath;
    if (m_sPresetPath.back() != '/')
    {
        m_sPresetPath.push_back('/');
    }
    m_sIniFile = m_sPresetPath + sPresetConfigFile;

    if (!read_profile_double("main", "robot_position_x", &dRobotPositionX, m_sIniFile.c_str()))
    {
        ROS_ERROR("read robot position x failed");
        ResultFailed("read robot position x failed");
        return;
    }

    if (!read_profile_double("main", "robot_position_y", &dRobotPositionY, m_sIniFile.c_str()))
    {
        ROS_ERROR("read robot position y failed");
        ResultFailed("read robot position y failed");
        return;
    }

    m_RobotStatusMutex.lock();
    /* 
    if (m_cNavigationStatus != 0x00)
    {
        ROS_ERROR("==>find_scale_cmd, trans_id:%s, but nav_status:%d abnormal, failed", m_sTransId.c_str(), m_cNavigationStatus);
        m_RobotStatusMutex.unlock();
        server.setSucceeded(result);
        return;
    }
   */
    if (fabs(dRobotPositionX - m_dRobotPositionX) >= m_dMaxOffsetDelta
        || fabs(dRobotPositionY - m_dRobotPositionY) >= m_dMaxOffsetDelta)
    {
        ROS_ERROR("robot preset position:[%.2f, %.2f], current position:[%.2f, %.2f]",
                  dRobotPositionX, dRobotPositionY, m_dRobotPositionX, m_dRobotPositionY);
        m_RobotStatusMutex.unlock();
        ResultFailed("robot position error");
        return;
    }

    m_bSamePose = fabs(m_dRobotPositionX - m_dLastRobotPositionX) <= m_dSamePositionDelta
                  && fabs(m_dRobotPositionY - m_dLastRobotPositionY) <= m_dSamePositionDelta
                  && fabs(m_dRobotOrientation - m_dLastRobotOrientation) <= m_dSameOrientationDelta;

    m_RobotStatusMutex.unlock();

//    ROS_INFO("==>find_scale_cmd, trans_id:%s, path:%s", m_sTransId.c_str(), sPresetPath.c_str());

    m_sCaptureTime = GetTimeString();
    m_sPictureFileName.clear();
    m_sInfraredFileName.clear();
    m_vsForeignPictureFileName.clear();

    SetWorkStatus(work);
    ExecuteThread();
}

void FindScale::NavStatusCB(const wootion_msgs::NavStatus::ConstPtr &Cmd)
{
    m_RobotStatusMutex.lock();
    m_dRobotPositionX = Cmd->position[0];
    m_dRobotPositionY = Cmd->position[1];
    m_dRobotOrientation = Cmd->orientation;
    m_RobotStatusMutex.unlock();
}

bool FindScale::CallControl(const CommandType &eCommand, const string &sInput, std::string &sOutput, const int nMaxRetry)
{
    int nRetryCount;

    for (nRetryCount = 1; nRetryCount <= nMaxRetry; nRetryCount++)
    {
        if (pCaller->Call(eCommand, sInput, sOutput))
        {
            return true;
        }
        ROS_WARN("CallControl failed, retry count:%d, command:%d, input:%s, output:%s",
                 nRetryCount, eCommand, sInput.c_str(), sOutput.c_str());
    }
    return false;
}

bool FindScale::ReadImage(string &sPicture, Mat &Image)
{
    Image = imread(sPicture, IMREAD_COLOR);//以灰度形式读取图像
    if (!Image.data)
    {
        ROS_INFO("ReadImage error while reading %s", sPicture.c_str());
        return false;
    }

    return true;
}

#if 0
double FindScale::GetPSNR(Mat &I1, Mat &I2)
{
    Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    Scalar s = sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double mse = sse / (double)(I1.channels() * I1.total());
        double psnr = 10.0*log10((255*255)/mse);
        return psnr;
    }
}
#endif

double FindScale::CalcImageAngle(const Mat &PresetImage, const Mat &CurrentImage, const Rect &RoiPreset,
                                 double dZoomLevel, double &dPanAngle, double &dTiltAngle, vector<int> &RoiVertex)
{
    double dResult;
    unsigned long ulVertexSize = ulMaxVertexSize;
    unsigned long i;
    int nVertex[ulMaxVertexSize] = {0};

    if (PresetImage.rows != CurrentImage.rows || PresetImage.cols != CurrentImage.cols)
    {
        dPanAngle = 0.0;
        dTiltAngle = 0.0;
        ROS_ERROR("image size is different");
        return 0;
    }

    cv::Point2f shift;
    dResult = calc_shift(PresetImage, CurrentImage, RoiPreset, shift);
//    ROS_INFO("visible shift:[%.2f, %.2f]", shift.x, shift.y);

    if (m_nInfrared == 0)
    {
        if (!read_profile_int_vector("other", "roi", ',', nVertex, &ulVertexSize, (m_sPresetPath + sPresetCalibFile).c_str()))
        {
            ROS_ERROR("read roi from calib file failed");
            return 0;
        }

//        for (i = 0; i < ulVertexSize; i++)
//        {
//            ROS_INFO("read roi vertex[%lu]:%d", i, nVertex[i]);
//        }

        if (ulVertexSize > ulMaxVertexSize || ulVertexSize == 0)
        {
            ROS_ERROR("roi vertex size:%lu abnormal", ulVertexSize);
            return 0;
        }

        RoiVertex.resize(ulVertexSize);

        for (i = 0; i < ulVertexSize; i++)
        {
            if (i % 2 == 0)
            {
                RoiVertex[i] = nVertex[i] + (int)round(shift.x);
            }
            else
            {
                RoiVertex[i] = nVertex[i] + (int)round(shift.y);
            }
        }

//        for (i = 0; i < ulVertexSize; i++)
//        {
//            ROS_INFO("visible vertex[%lu]:%d", i, RoiVertex[i]);
//        }
    }

    double err_x_mean = -shift.x;
    double err_y_mean = -shift.y;
    dPanAngle = atan2(err_x_mean / PresetImage.cols * camera_sensor_w, dZoomLevel * 4.5);
    dTiltAngle = atan2(err_y_mean / PresetImage.rows * camera_sensor_h, dZoomLevel * 4.5);
    return dResult;
}

bool FindScale::SetCameraPreset(int nLevel)
{
    string sOutput;
    string sLevel;

    sLevel = "zoom" + to_string(nLevel);

    if (!read_profile_double(sLevel.c_str(), "zoom_level", &m_dZoomLevel, m_sIniFile.c_str()))
    {
        ROS_ERROR("read zoom level failed");
        return false;
    }

    if (m_dZoomLevel < m_dPreZoomLevel)
    {
        m_dPreZoomLevel = m_dZoomLevel;
    }

    if (!CallControl(set_zoom_level, std::to_string(m_dZoomLevel), sOutput))
    {
        ROS_ERROR("set zoom level:%.1f failed, output:%s", m_dZoomLevel, sOutput.c_str());
        return false;
    }

    if (!read_profile_int(sLevel.c_str(), "wdr_level", &m_nWdrLevel, m_sIniFile.c_str()))
    {
        ROS_ERROR("read wdr level failed");
        return false;
    }

    if (!CallControl(set_wdr_level, std::to_string(m_nWdrLevel), sOutput))
    {
        ROS_ERROR("set wdr level:%d failed, output:%s", m_nWdrLevel, sOutput.c_str());
        return false;
    }

    if (!read_profile_int(sLevel.c_str(), "min_focus_distance", &m_nMinFocusDistance, m_sIniFile.c_str()))
    {
        ROS_ERROR("read min focus distance failed");
        return false;
    }

    if (!CallControl(set_min_focus_distance, std::to_string(m_nMinFocusDistance), sOutput))
    {
        ROS_ERROR("set min focus distance:%d failed, output:%s", m_nMinFocusDistance, sOutput.c_str());
        return false;
    }

    if (!read_profile_int(sLevel.c_str(), "focus_position", &m_nFocusPosition, m_sIniFile.c_str()))
    {
        ROS_ERROR("read focus position failed");
        return false;
    }

    if (!CallControl(set_focus_position, std::to_string(m_nFocusPosition), sOutput))
    {
        ROS_ERROR("set focus position:%d failed, output:%s", m_nFocusPosition, sOutput.c_str());
        return false;
    }

    if (read_profile_int(sLevel.c_str(), "wait_time", &m_nWaitTime, m_sIniFile.c_str()))
    {
        this_thread::sleep_for(chrono::milliseconds(m_nWaitTime));
    }

//    ROS_INFO("zoom_level:%.2f, min_focus_distance:%d, focus_position:%d, wdr_level:%d",
//             m_dZoomLevel, m_nMinFocusDistance, m_nFocusPosition, m_nWdrLevel);

    return true;
}

bool FindScale::SetTerracePreset(bool bPreRotate)
{
    string sInput;
    string sOutput;
    int nRobotYawAngle;
    int nYawAngle;
    int nPitchAngle;

    if (bPreRotate)
    {
        nRobotYawAngle = (int)round(m_dPreRobotOrientation * 18000 / M_PI);
    }
    else
    {
        m_RobotStatusMutex.lock();
        nRobotYawAngle = (int)round(m_dRobotOrientation * 18000 / M_PI);
        m_RobotStatusMutex.unlock();
    }

    if (!read_profile_int("terrace", "yaw_angle", &nYawAngle, m_sIniFile.c_str()))
    {
        ROS_ERROR("read terrace yaw angle failed");
        return false;
    }

    if (!read_profile_int("terrace", "pitch_angle", &nPitchAngle, m_sIniFile.c_str()))
    {
        ROS_ERROR("read terrace pitch angle failed");
        return false;
    }

    m_nPanAngle = nYawAngle - nRobotYawAngle;
    AdjustYawAngle(m_nPanAngle);
    m_nTiltAngle = nPitchAngle;
    m_nPresetYawAngle = nYawAngle;
    m_nPresetPitchAngle = nPitchAngle;

    m_bSimilarAngle = abs(m_nPresetYawAngle - m_nLastPresetYawAngle) <= m_nSimilarAngleDelta
                      && abs(m_nPresetPitchAngle - m_nLastPresetPitchAngle) <= m_nSimilarAngleDelta;

    if (m_bSamePose && m_bSimilarAngle && m_dLastResult >= 0.95)
    {
        ROS_INFO("same pose and similar angle, adjust preset from [%d, %d] to [%d, %d]",
                 m_nPanAngle, m_nTiltAngle, m_nPanAngle + m_nLastRotatePanAngle, m_nTiltAngle + m_nLastRotateTiltAngle);
        m_nPanAngle -= m_nLastRotatePanAngle;
        m_nTiltAngle -= m_nLastRotateTiltAngle;
        AdjustYawAngle(m_nPanAngle);
        AdjustPitchAngle(m_nTiltAngle);
    }

    sInput = std::to_string(m_nPanAngle) + "," + std::to_string(m_nTiltAngle);

    if (!CallControl(set_angle, sInput, sOutput))
    {
        ROS_WARN("set angle:%s failed, output:%s", sInput.c_str(), sOutput.c_str());
    }

    return true;
}

bool FindScale::SetArmPreset()
{
    string sInput;
    string sOutput;
    char szValue[MAX_VALUE_SIZE];
    vector<string> vsResult;

    if (!read_profile_string("arm", "arm_file", szValue, MAX_VALUE_SIZE, m_sIniFile.c_str()))
    {
        ROS_ERROR("read arm position failed");
        return false;
    }

    if(strcmp(szValue,"null") != 0)
    {
        sInput = m_sPresetPath + sArmOrbitFile;
        sInput += ",+";

        if (!CallControl(play_orbit, sInput, sOutput))
        {
            ROS_WARN("play arm orbit failed, input:%s, output:%s", sInput.c_str(), sOutput.c_str());
        }

        this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!read_profile_string("arm", "orientation", szValue, MAX_VALUE_SIZE, m_sIniFile.c_str()))
    {
        ROS_ERROR("read arm orientation failed");
        return false;
    }

    sInput = szValue;

    if (!CallControl(set_orientation, sInput, sOutput))
    {
        ROS_WARN("set arm orientation failed, input:%s, output:%s", sInput.c_str(), sOutput.c_str());
        return false;
    }

    this_thread::sleep_for(std::chrono::milliseconds(200));
	
	vsResult = SplitString(sInput, ",");
    m_nPanAngle = (int)round(stoi(vsResult[2]));
    m_nTiltAngle = (int)round(stoi(vsResult[1]));

    return true;
}

string FindScale::GetTimeString()
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

double FindScale::FindByCamera(int nLevel)
{
    string sPresetFile;
    string sCurrentFile;
    string sTempFile;
    string sRoiFile;
    string sInput;
    string sOutput;
    string sLevel;
    int nStepCount;
    double dResult = 0;
    double dPanAngle;
    double dTiltAngle;
    double dMaxRotatePanAngle;
    double dMaxRotateTiltAngle;
    int nYawAngle;
    int nRotatePanAngle;
    int nRotateTiltAngle;
    int nRotatePanAngleLast = 0;
    int nRotateTiltAngleLast = 0;
    ros::Time StartTime;
    long lDuration;
    Mat PresetImage;
    Mat CurrentImage;
    Rect PresetRoi = Rect(0, 0, 0, 0);

    sLevel = "zoom" + to_string(nLevel);
    sPresetFile = m_sPresetPath + sLevel + ".jpg";    //预置位图片
    sRoiFile = m_sPresetPath + sLevel + ".xml";       //预置位标注

    ROS_INFO("start to find target at level:%d, current angle:[%d, %d]", nLevel, m_nPanAngle, m_nTiltAngle);

    if (!SetCameraPreset(nLevel))
    {
        ROS_WARN("set camera to preset failed, zoom:%.1f", m_dZoomLevel);
    }
    else
    {
        ROS_INFO("set camera to preset success, zoom:%.1f", m_dZoomLevel);
    }

    StartTime = ros::Time::now();

    if (!ReadImage(sPresetFile, PresetImage))
    {
        ROS_ERROR("read preset image failed");
        return dResult;
    }

    if (!WtAnnotation::LoadRoi(sRoiFile, PresetRoi))
    {
        ROS_INFO("load roi from file:%s failed", sRoiFile.c_str());
    }

    for (nStepCount = 0;; nStepCount++)
    {
        if (GetWorkStatus() == stop)
        {
            dResult = 1.0;
            break;
        }

        lDuration = (ros::Time::now() - StartTime).toNSec() / 1000000000;

        if (lDuration >= m_nLevelFindTimeout)
        {
            ROS_WARN("find target on %s time out", sLevel.c_str());
            break;
        }

        if (nStepCount == m_nMaxStep[nLevel])
        {
            ROS_INFO("rotate terrace step:%d too much, found", nStepCount);
            dResult = 0.3;
            break;
        }

        sTempFile = sLevel + "_" + m_sCaptureTime + format("_%02d.jpg", nStepCount);
        sCurrentFile = m_sPresetPath + sTempFile;

        if (!CallControl(capture_picture, sCurrentFile, sOutput))
        {
            ROS_ERROR("capture picture:%s failed, output:%s", sCurrentFile.c_str(), sOutput.c_str());
            break;
        }

        if (!ReadImage(sCurrentFile, CurrentImage))
        {
            ROS_ERROR("read temp image failed");
            break;
        }

        //update picture file name after capture picture and read image success,
        //if failed, picture file name will be the last time's picture file name
        m_sPictureFileName = sTempFile;

        dResult = CalcImageAngle(PresetImage, CurrentImage, PresetRoi, m_dZoomLevel, dPanAngle, dTiltAngle, m_vnRoiVertex);
        if (dResult == 0)
        {
            ROS_ERROR("CalcImageAngle failed on picture:%s", sCurrentFile.c_str());
            break;
        }

//        ROS_INFO("calculate rotate angle result:[%d, %d]", (int)round(dPanAngle * 18000 / M_PI), (int)round(dTiltAngle * 18000 / M_PI));

#if 0
        dMaxRotatePanAngle = 0.05 * 63 / m_dZoomLevel;
        dMaxRotateTiltAngle = 0.05 * 35 / m_dZoomLevel;
        if (fabs(dPanAngle) < dMaxRotatePanAngle && fabs(dTiltAngle) < dMaxRotateTiltAngle)
        {
            ROS_INFO("current zoom threshold:[pan_max:%.2f), tilt_max:%.2f)] too small, found",
                     dMaxRotatePanAngle, dMaxRotateTiltAngle)
            return true;
        }
#endif

        nRotatePanAngle = (int)round(2 * atan(tan(dPanAngle / 2) / cos(m_nTiltAngle * M_PI / 18000)) * 18000 / M_PI);
        nRotateTiltAngle = (int)round(dTiltAngle * 18000 / M_PI);

        ROS_INFO("step:%d, rotate angle:[pan:%d, tilt:%d]", nStepCount, nRotatePanAngle, nRotateTiltAngle);

        if (abs(nRotatePanAngle) <= m_nMinRotatePanAngle && abs(nRotateTiltAngle) <= m_nMinRotateTiltAngle)
        {
            ROS_INFO("angle too small to rotate, found");
            break;
        }

        if (abs(nRotatePanAngleLast - nRotatePanAngle) <= m_nMinDeltaRotateAngle
            && abs(nRotateTiltAngleLast - nRotateTiltAngle) <= m_nMinDeltaRotateAngle)
        {
            ROS_INFO("angle is going to be converged, found");
            break;
        }

        if (m_nTerraceType == 0)
        {
            m_nPanAngle += nRotatePanAngle;
            m_nTiltAngle -= nRotateTiltAngle;
            AdjustYawAngle(m_nPanAngle);
            AdjustPitchAngle(m_nTiltAngle);

            sInput = std::to_string(m_nPanAngle) + "," + std::to_string(m_nTiltAngle);

            if (!CallControl(set_angle, sInput, sOutput))
            {
                ROS_WARN("set angle:%s failed, output:%s", sInput.c_str(), sOutput.c_str());
            }

            ROS_INFO("set angle:[%d, %d] finished", m_nPanAngle, m_nTiltAngle);
        }
        else if (m_nTerraceType == 1)
        {
            m_nPanAngle -= nRotatePanAngle;
            m_nTiltAngle -= nRotateTiltAngle;
            sInput = "0," + std::to_string(m_nTiltAngle) + "," + std::to_string(m_nPanAngle);

            if (!CallControl(set_orientation, sInput, sOutput))
            {
                ROS_WARN("set arm orientation failed, input:%s, output:%s", sInput.c_str(), sOutput.c_str());
            }

            ROS_INFO("rotate arm orientation:[0, %d, %d] finished", m_nPanAngle, m_nTiltAngle);
        }

//        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        nRotatePanAngleLast = nRotatePanAngle;
        nRotateTiltAngleLast = nRotateTiltAngle;
    }

    if (m_nForeignDetect != 0)
    {
        m_vsForeignPictureFileName.emplace_back(m_sPictureFileName);
    }

    if (m_nTerraceType == 0)
    {
        m_RobotStatusMutex.lock();
        nYawAngle = (int)round(m_dRobotOrientation * 18000 / M_PI) + m_nPanAngle;
        m_RobotStatusMutex.unlock();
        AdjustYawAngle(nYawAngle);

        m_nRotatePanAngle = m_nPresetYawAngle - nYawAngle;
        m_nRotateTiltAngle = m_nPresetPitchAngle - m_nTiltAngle;
        m_dLastResult = dResult;
    }

    ROS_INFO("find by camera at level:%d finished, result:%f", nLevel, dResult);
    return dResult;
}


bool FindScale::CalcInfraredVertex(const std::string &sCurrentPicture, std::vector<int> &Vertex)
{
    int nVertex[ulMaxVertexSize] = {0};
    unsigned long i;
    unsigned long ulVertexSize = ulMaxVertexSize;
    std::string preset_xml_path = m_sPresetPath + "zoom0" + "_infrared.xml";
    std::string preset_arr_path = m_sPresetPath + "zoom0" + "_infrared";

    Mat img_preset = read_infrared_array::read_as_gray(preset_arr_path);
    if (!img_preset.data)
    {
        ROS_ERROR("error while reading %s", preset_arr_path.c_str());
        return false;
    }

    Mat img_run = read_infrared_array::read_as_gray(sCurrentPicture);
    if (!img_run.data)
    {
        ROS_ERROR("error while reading %s", sCurrentPicture.c_str());
        return false;
    }

    Rect roi_preset = Rect(0, 0, 0, 0);
    if (!WtAnnotation::LoadRoi(preset_xml_path, roi_preset))
    {
        ROS_INFO("load roi from file:%s failed", preset_xml_path.c_str());
    }

    cv::Point2f shift;
    calc_shift(img_preset, img_run, roi_preset, shift);
//    ROS_INFO("infrared shift:[%.2f, %.2f]", shift.x, shift.y);

    if (!read_profile_int_vector("other", "prebox", ',', nVertex, &ulVertexSize, (m_sPresetPath + sPresetCalibFile).c_str()))
    {
        ROS_ERROR("read infrared prebox failed");
        return false;
    }

//    for (i = 0; i < ulVertexSize; i++)
//    {
//        ROS_INFO("read prebox vertex[%lu]:%d", i, nVertex[i]);
//    }

    if (ulVertexSize > ulMaxVertexSize || ulVertexSize == 0)
    {
        ROS_ERROR("infrared prebox vertex size:%lu abnormal", ulVertexSize);
        return false;
    }

    Vertex.resize(ulVertexSize);

    for (i = 0; i < ulVertexSize; i++)
    {
        if (i % 2 == 0)
        {
            Vertex[i] = nVertex[i] + (int)round(shift.x);
        }
        else
        {
            Vertex[i] = nVertex[i] + (int)round(shift.y);
        }
    }

//    for (i = 0; i < ulVertexSize; i++)
//    {
//        ROS_INFO("infrared vertex[%lu]:%d", i, Vertex[i]);
//    }

    return true;
}

bool FindScale::FindTarget()
{
    string sOutput;
    string sForeignFileName;
    int i;
    ros::Time StartTime;
    long lDuration;
    double dResult;
    bool bOffset = false;

    if (!read_profile_int("main", "infrared", &m_nInfrared, m_sIniFile.c_str()))
    {
        ROS_ERROR("read infrared failed");
        return false;
    }

    if (!CallControl(set_focus_mode, "manual", sOutput))
    {
        ROS_ERROR("set focus mode to manual failed, output:%s", sOutput.c_str());
        return false;
    }

    if (m_nTerraceType == 0)
    {
        if (!SetTerracePreset(false))
        {
            ROS_WARN("set terrace to preset failed");
        }
        else
        {
            ROS_INFO("set terrace to preset success");
        }
    }
    else if (m_nTerraceType == 1)
    {
        if (!SetArmPreset())
        {
            ROS_WARN("set arm to preset failed");
        }
        else
        {
            ROS_INFO("set arm to preset success");
        }
    }

    if (m_nInfrared == 1)
    {
        m_nEndLevel = 0;
        dResult = FindByCamera(m_nEndLevel);

        if (GetWorkStatus() == stop)
        {
            return true;
        }

        if (dResult == 0)
        {
            ROS_ERROR("find target for infrared failed");
            return false;
        }

        m_sInfraredFileName = "zoom" + to_string(m_nEndLevel) + "_" + m_sCaptureTime + "_infrared";
        if (!CallControl(save_temperature, m_sPresetPath + m_sInfraredFileName, sOutput))
        {
            ROS_ERROR("save_temperature:%s failed, output:%s", (m_sPresetPath + m_sInfraredFileName).c_str(), sOutput.c_str());
            return false;
        }

        if (!CalcInfraredVertex(m_sPresetPath + m_sInfraredFileName, m_vnRoiVertexInfrared))
        {
            ROS_INFO("CalcInfraredVertex failed");
        }

        ROS_INFO("find target for infrared success");
        return true;
    }
    else
    {
        if (!read_profile_int("main", "levels", &m_nPresetLevels, m_sIniFile.c_str()))
        {
            ROS_ERROR("read levels failed");
            return false;
        }

        if (!read_profile_int("main", "end_level", &m_nEndLevel, m_sIniFile.c_str()))
        {
            m_nEndLevel = 0;
        }

        StartTime = ros::Time::now();

        if (m_nEnvironmentType == 1 && m_nTerraceType == 0)
        {
            if (m_bSamePose && m_bSimilarAngle && m_dLastResult >= 0.95
                && abs(m_nLastRotatePanAngle) <= m_nOneShotMaxRotateAngle
                && abs(m_nLastRotateTiltAngle) <= m_nOneShotMaxRotateAngle)
            {
                ROS_INFO("trying to use one shot, last rotate angle:[%d, %d]", m_nLastRotatePanAngle, m_nLastRotateTiltAngle);

                dResult = FindByCamera(0);

                if (GetWorkStatus() == stop)
                {
                    return true;
                }

                if (dResult >= 0.7)
                {
                    if (abs(m_nRotatePanAngle) <= m_nOneShotMaxRotateAngle
                        && abs(m_nRotateTiltAngle) <= m_nOneShotMaxRotateAngle
                        && abs(m_nRotatePanAngle - m_nLastRotatePanAngle) <= m_nMaxRotateAngleDelta
                        && abs(m_nRotateTiltAngle - m_nLastRotateTiltAngle) <= m_nMaxRotateAngleDelta)
                    {
                        ROS_INFO("one shot success, rotate angle:[%d, %d]", m_nRotatePanAngle, m_nRotateTiltAngle);
                        return true;
                    }
                    else
                    {
                        ROS_WARN("one shot failed, rotate angle:[%d, %d]", m_nRotatePanAngle, m_nRotateTiltAngle);
                    }
                }
                else
                {
                    ROS_WARN("one shot failed, result:%f to small]", dResult);
                }
            }
        }

        i = m_nPresetLevels - 1;

        while (i >= m_nEndLevel)
        {
            lDuration = (ros::Time::now() - StartTime).toNSec() / 1000000000;
            if (lDuration >= m_nMaxFindTimeout)
            {
                ROS_WARN("find target max time out");
                return false;
            }

            dResult = FindByCamera(i);

            if (GetWorkStatus() == stop)
            {
                return true;
            }

            if (dResult == 0)
            {
                ROS_WARN("find target failed at level:%d", i);
                return false;
            }
            else
            {
                if (i == m_nEndLevel)
                {
                    break;
                }

                if (dResult > 0.5 && m_bSkipLevel && i - 2 >= m_nEndLevel) //returned confidence more than 0.5, skip next level if possible
                {
                    i -= 2;
                }
                else //returned confidence less than 0.5, continue with next level
                {
                    i--;
                }
            }
        }

        if (m_nForeignDetect != 0)
        {
            if (m_dPreZoomLevel != 1.0)
            {
                if (!CallControl(set_zoom_level, std::to_string(1.0), sOutput))
                {
                    ROS_ERROR("set zoom level:1.0 for foreign detect failed, output:%s", sOutput.c_str());
                    return false;
                }

                sForeignFileName = "foreign_" + m_sCaptureTime + ".jpg";
                if (!CallControl(capture_picture, m_sPresetPath + sForeignFileName, sOutput))
                {
                    ROS_ERROR("capture picture:%s for foreign detect failed, output:%s", sForeignFileName.c_str(), sOutput.c_str());
                    return false;
                }

                m_vsForeignPictureFileName.emplace_back(sForeignFileName);
            }
        }

        if (m_nSaveInfraredPicture == 1)
        {
            m_sInfraredFileName = "zoom" + to_string(m_nEndLevel) + "_" + m_sCaptureTime + "_infrared.jpg";
            if (!CallControl(infrared_capture_picture, m_sPresetPath + m_sInfraredFileName, sOutput))
            {
                ROS_ERROR("capture_picture_infrared:%s failed, output:%s", (m_sPresetPath + m_sInfraredFileName).c_str(), sOutput.c_str());
                return false;
            }
        }

        if (m_nEndLevel != 0)
        {
            return SetCameraPreset(0);
        }

        return true;
    }
}

WorkStatus FindScale::GetWorkStatus()
{
    return m_eWorkStatus;
}

void FindScale::SetWorkStatus(WorkStatus eWorkStatus)
{
    m_eWorkStatus = eWorkStatus;
}

void FindScale::ExecuteThread()
{
    std::unique_lock<std::mutex> lock(m_Mutex);
    m_bExecute = true;
    m_Condition.notify_one();
}
}