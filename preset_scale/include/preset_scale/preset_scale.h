///*
// * preset.h
// *
// *  Created on: Aug 3, 2018
// *      Author: LaiHan
// *  Modified on: Aug 13, 2018
// *      Author: JiangXiaolu
// *  Modified on: Dec 27, 2018
// *      Author: LaiHan
// */

#ifndef _PRESET_H_
#define _PRESET_H_

#define __cplusplus 201103L

#include <sys/stat.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <atomic>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "ini_file.h"
#include "wootion_msgs/PresetScaleCmd.h"
#include "wootion_msgs/PresetScaleAck.h"
#include "wootion_msgs/NavStatus.h"
#include "service_caller.h"

using namespace std;
using namespace cv;
using namespace ServiceCaller;

namespace PresetScale
{

static const char cNavWarnFlag = 0x02;
static const string sArmOrbitFile = "arm_orbit.txt";
static const string sArmOrbitFixedPathName = "/home/ros/track/arm_orbit.txt";

class PresetScale
{
public:
    PresetScale();
    ~PresetScale();
    void AdjustYawAngle(int &nAngle);
    string GetTimeString();
    bool CreateDir(const char *sPathName);
    bool SaveMainIni();
    bool SaveTerraceIni();
    bool SaveArmIni();
    bool SaveCameraIni(int nLevel, double dZoomLevel);
    void PublishPresetScaleAck(wootion_msgs::PresetScaleAck &PresetAck);
    void PresetCmdCallBack(const wootion_msgs::PresetScaleCmd::ConstPtr &PresetCmd);
    bool ServiceFunc(wootion_msgs::ControlService::Request &Req, wootion_msgs::ControlService::Response &Resp);
    void PresetScaleProc(std::string &sResult);
    void NavStatusCallBack(const wootion_msgs::NavStatus::ConstPtr &NavStatus);
private:
    string m_sNodeName;
    string m_sPresetPath;
    string m_sPresetFile;
    string m_sIniFile;
    string m_sDirectory;
    int m_nTerraceType; //0:cloud terrace; 1:arm
    double m_dPresetZoomLevel;
    double m_dMaxZoomLevel;
    double m_dMinZoomLevel;
    double m_dMiddleZoomLevel;
    double m_dInfraredZoomLevel;
    int m_nInfrared;
    string m_sType;
    string m_sExtraData;
    int m_nLevelCount;
    double m_dRobotPositionX;
    double m_dRobotPositionY;
    double m_dRobotOrientation;
    unsigned char m_cNavigationStatus;
    mutex m_RobotStatusMutex;
    atomic_bool m_bBusy;
    string m_sPresetId;
    string m_sRecordTime;
    string m_sPresetScaleCmdTopic;
    string m_sPresetScaleAckTopic;
    string m_sNavStatusTopic;
    string m_sCameraService;
    string m_sTerraceService;
    string m_sInfraredService;
    string m_sPresetService;
    string m_sArmService;
    ros::NodeHandle m_PublicNodeHandle;
    ros::NodeHandle m_PrivateNodeHandle;
    ros::Subscriber m_PresetCmdSub;
    ros::Publisher m_PresetAckPub;
    ros::Subscriber m_NavStatusSub;
    ros::ServiceServer m_Service;
    Caller *pCaller;
};

}

#endif /* _PRESET_H_ */
