#ifndef FIND_SCALE_H
#define FIND_SCALE_H

#define __cplusplus 201103L

#include <cstdlib>
#include <thread>
#include <string>
#include <cmath>
#include <atomic>
#include <deque>
#include <condition_variable>
#include <geometry_msgs/Point.h>
#include <actionlib/server/simple_action_server.h>
#include "wootion_msgs/GeneralAction.h"
#include "wootion_msgs/NavStatus.h"
#include "wootion_msgs/GeneralTopic.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include "ros/ros.h"
#include "ini_file.h"
#include "calc_shift.h"
#include "WtAnnotation.h"
#include "service_caller.h"
#include "read_infrared_array.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

namespace FindScale
{
//1080p:12.8:7.2
//1280X960:12.8:9.6
//CMOS:12.8:9.6
static const double camera_sensor_w = 12.8/2.8;
static const double camera_sensor_h = 7.2/2.8;

//static const int img_w = 480;
//static const int img_h = 277;

static const std::string sPresetConfigFile = "preset.ini";
static const std::string sPresetCalibFile = "preset_calib_info.ini";
static const std::string sArmOrbitFile = "arm_orbit.txt";
static const unsigned long ulMaxVertexSize = 8;

typedef enum _work_status_
{
    idle,
    stop,
    work,
}WorkStatus;

class FindScale
{
public:
    using Server=actionlib::SimpleActionServer<wootion_msgs::GeneralAction>;
    FindScale();
    ~FindScale();
    void AdjustYawAngle(int &nAngle);
    void AdjustPitchAngle(int &nAngle);
    void FindScaleThreadFunc();
    void FindScaleCmdCB(const std::string& sPresetPath);
    void NavStatusCB(const wootion_msgs::NavStatus::ConstPtr &Cmd);
    void ArmHeartBeatSub(const wootion_msgs::GeneralTopic::ConstPtr &ArmMsg);
    bool CallControl(const ServiceCaller::CommandType &eCommand, const std::string &sInput, std::string &sOutput, const int nMaxRetry = 3);
    bool ReadImage(std::string &sPicture, cv::Mat &Image);
    double CalcImageAngle(const cv::Mat &PresetImage, const cv::Mat &CurrentImage, const cv::Rect &RoiPreset, double dZoomLevel, double &dPanAngle, double &dTiltAngle, std::vector<int> &RoiVertex);
    bool SetCameraPreset(int nLevel);
    bool SetTerracePreset(bool bPreRotate);
    bool SetArmPreset();
    std::string GetTimeString();
    double FindByCamera(int nLevel);
    bool CalcInfraredVertex(const std::string &sCurrentPicture, std::vector<int> &Vertex);
    bool FindTarget();
    WorkStatus GetWorkStatus();
    void SetWorkStatus(WorkStatus eWorkStatus);
    void ExecuteThread();
    void ExecuteActionCb(const wootion_msgs::GeneralGoalConstPtr &goal);
    void PreemptCb();
    bool PreFind(const std::string &sPresetPath);
    void ResultSucceed();
    void ResultFailed(const std::string &sReason);
private:
    int m_nPanAngle;
    int m_nTiltAngle;
    int m_nPresetYawAngle;
    int m_nPresetPitchAngle;
    int m_nLastPresetYawAngle;
    int m_nLastPresetPitchAngle;
    int m_nRotatePanAngle;
    int m_nRotateTiltAngle;
    int m_nLastRotatePanAngle;
    int m_nLastRotateTiltAngle;
    double m_dLastResult;
    double m_dRobotPositionX;
    double m_dRobotPositionY;
    double m_dRobotOrientation;
    double m_dLastRobotPositionX;
    double m_dLastRobotPositionY;
    double m_dLastRobotOrientation;
    double m_dPreRobotOrientation;
    double m_dPreZoomLevel;
    double m_dMaxOffsetDelta;
    unsigned char m_cNavigationStatus;
    std::mutex m_RobotStatusMutex;
    double m_dZoomLevel;
    int m_nMinFocusDistance;
    int m_nFocusPosition;
    int m_nWdrLevel;
    int m_nLevelFindTimeout;
    int m_nMaxFindTimeout;
    int m_nFindWaitTime;
    int m_nMaxStep[3];
    int m_nInfrared;
    int m_nPresetLevels;
    int m_nEndLevel;
    int m_nWaitTime;
    int m_nEnvironmentType;
    int m_nOneShotMaxRotateAngle;
    int m_nMaxRotateAngleDelta;
    bool m_bSamePose;
    bool m_bSimilarAngle;
    double m_dSamePositionDelta;
    double m_dSameOrientationDelta;
    int m_nSimilarAngleDelta;
    int m_nSaveInfraredPicture;
    int m_nTerraceType; //0:cloud terrace; 1:arm
    std::string m_sNodeName;
    std::string m_sSender;
    std::string m_sCameraService;
    std::string m_sTerraceService;
    std::string m_sInfraredService;
    std::string m_sArmService;
    std::string m_sFindScaleCmdTopic;
    std::string m_sFindScaleAckTopic;
    std::string m_sNavStatusTopic;
    std::string m_sArmHeartBeatTopic;
    std::string m_sPresetFile;
    std::string m_sPresetPath;
    std::string m_sCaptureTime;
    std::string m_sPictureFileName;
    std::string m_sInfraredFileName;
    std::string m_sArmStatus;
    std::vector<std::string> m_vsForeignPictureFileName;
    int m_nForeignDetect;
    std::vector<int> m_vnRoiVertex;
    std::vector<int> m_vnRoiVertexInfrared;
    int m_nMinRotatePanAngle;
    int m_nMinRotateTiltAngle;
    int m_nMinDeltaRotateAngle;
    bool m_bSkipLevel;
    std::string m_sIniFile;
    ros::NodeHandle m_PrivateNodeHandle;
    ros::NodeHandle m_PublicNodeHandle;
    ros::Subscriber m_FindScaleCmdSub;
    ros::Publisher m_FindScaleAckPub;
    ros::Subscriber m_NavStatusSub;
    ros::Subscriber m_ArmHeartBeatSub;
    std::string m_sTransId;
    std::thread *m_pFindScaleThread;
    std::atomic<WorkStatus> m_eWorkStatus;
    bool m_bExecute;
    std::condition_variable m_Condition;
    std::mutex m_Mutex;
    std::string m_sGoalType;
    bool m_bActionLive;
    Server m_ActionlibServer;
    ServiceCaller::Caller *pCaller;
};

}


#endif /* FIND_SCALE_H */
