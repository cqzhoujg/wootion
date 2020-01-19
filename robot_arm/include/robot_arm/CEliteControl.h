//
// Created by zhoujg on 19-12-6.
//

#ifndef PROJECT_CELITECONTROL_H
#define PROJECT_CELITECONTROL_H
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
#include <math.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "ros/ros.h"
#include "elt_ctx.h"
#include "elt_robot.h"
#include "elt_rpc.h"
#include "cJSON.h"
#include "elt_transport.h"
#include "elt_define.h"
#include "wootion_msgs/GeneralCmd.h"
#include "wootion_msgs/GeneralAck.h"
#include "wootion_msgs/ControlService.h"
#include "wootion_msgs/GeneralTopic.h"
#include "CFileRW.h"
#include "CArmExtraCtrl.h"

using namespace std;

typedef enum _tagEltStatus
{
    STOP = 0,
    PAUSE = 1,
    EMERGENCY_STOP = 2,
    MOVING = 3,
    ALARM = 4,
    DROP_LINE = 5,
}EltStatus;

const static vector<string> vsArmStatus = {
    "stop",
    "pause",
    "emergency_stop",
    "moving",
    "alarm",
    "drop_line",
};

typedef enum _tagDragStatus
{
    DISABLE,
    ENABLE,
    FORWARD,
    REVERSE
}DragStatus;

const static vector<string> vsDragStatus = {
    "disable",
    "enable",
};

typedef struct _tagEltPos
{
    elt_robot_pos eltPos={0};
} EltPos, *PEltPos;

typedef enum _tagAxisLimit
{
    AXIS_ONE_MAX,
    AXIS_TWO_MAX,
    AXIS_THREE_MAX,
    AXIS_FOUR_MAX,
    AXIS_FIVE_MAX,
    AXIS_SIX_MAX,
    
    AXIS_ONE_MIN,
    AXIS_TWO_MIN,
    AXIS_THREE_MIN,
    AXIS_FOUR_MIN,
    AXIS_FIVE_MIN,
    AXIS_SIX_MIN,
}AxisLimit;

typedef enum _tagAxisNum
{
    AXIS_ONE,
    AXIS_TWO,
    AXIS_THREE,
    AXIS_FOUR,
    AXIS_FIVE,
    AXIS_SIX,
}AxisNum;

typedef enum _tagRotateType
{
    AXIS_PITCH = AXIS_FOUR,
    AXIS_YAW = AXIS_FIVE,
    AXIS_ROLL = AXIS_SIX,
}RotateType;

/*name      max        min   度(°)

1	177.0	-177.0
2	177.0	-177
3	160.0	-160.0
4	80.0	-260.0
5	357.0	 3.0
6	177.0	-177.0*/
const static double AxisLimitAngle[12] = {
    175.0,  175.0,  158.0,   58.0, 355.0,  175.0,
   -175.0, -175.0, -158.0, -258.0,   5.0, -175.0
};

static vector<string> SplitString(const string &sInput, const string &sDelimiter)
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

class CEliteControl
{
public:
    CEliteControl();
    ~CEliteControl();

    bool Init();
    void UnInit();
    void UpdateEliteThreadFunc();
    void HeartBeatThreadFunc();
    void ControlThreadFunc();
    int UpdateEltOrigin();
    int EliteJointMove(elt_robot_pos &targetPos, string &sErr);
    int EliteMultiPointMove(elt_robot_pos &targetPos, string &sErrMsg);
    int GetElitePos(elt_robot_pos &pos_array);
    int EliteStop(string sErr="");
    int EliteDrag(int nCmd);
    int EliteRunDragTrack(const string &sFileName, double dSpeed, int nDirection, string &sErrMsg);
    bool EliteGotoOrigin(const string &sInput, string &sOutput);
    int EliteSyncMotorStatus();
    int EliteClearAlarm();
    void PrintJointData(elt_robot_pos &pos_array, string sFunName);
    bool ArmServiceFunc(wootion_msgs::ControlService::Request &Req, wootion_msgs::ControlService::Response &Resp);
    void ArmCmdCallBack(const wootion_msgs::GeneralCmd::ConstPtr &TerraceCmd);
    bool ArmOperation(const std::string &sCommand, const std::string &sInput, std::string &sOutput);

private:
    string m_sSerialPort;
    string m_sEliteRobotIP;
    string m_sPrintLevel;
    string m_sArmTrackPath;
    string m_sArmService;
    string m_sArmCmdTopic;
    string m_sArmAckTopic;
    string m_sHeartBeatTopic;
    string m_sAbnormalTopic;
    string m_sOrbitFileName;
    string m_sTaskName;
    string m_sResetOrbitFile;

    int m_nSerialSpeed;
    int m_nElitePort;
    int m_nEliteState;
    int m_nCheckReceiver;
    int m_nDragStatus;

    bool m_bRecordDragTrack;
    bool m_bBusy;

    double m_dEltSpeed;//百分比

    timespec m_tRecordDataTime;

    ELT_CTX m_eltCtx;
    elt_robot_pos m_EliteCurrentPos;
    elt_robot_pos m_DragOriginPos;
    elt_robot_pos m_EltOriginPos;
    std::deque<EltPos> m_dDragTrackDeque;

    std::thread *m_pEliteStatusThread;
    std::thread *m_pHeartBeatThread;
    std::thread *m_pControlThread;

    ros::Subscriber m_ArmCmdSubscriber;
    ros::Publisher m_ArmAckPublisher;
    ros::Publisher m_HeartBeatPublisher;
    ros::Publisher m_AbnormalPublisher;

    ros::ServiceServer m_ArmService;

    CFileRW m_TrackFile;
    CArmExtraCtrl m_ExtraCtrl;
};

#endif //PROJECT_CELITECONTROL_H
