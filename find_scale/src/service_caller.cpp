//
// Created by LaiHan on 19-12-17.
//

#include "service_caller.h"

namespace ServiceCaller
{

Caller::Caller(std::vector <std::string> vsServiceName):
    m_NodeHandle(),
    m_sCameraService(vsServiceName[0]),
    m_sTerraceService(vsServiceName[1]),
    m_sInfraredService(vsServiceName[2]),
    m_sArmService(vsServiceName[3])
{
    ros::Time::init();
    m_CameraClient = m_NodeHandle.serviceClient<wootion_msgs::ControlService>(m_sCameraService);
    m_TerraceClient = m_NodeHandle.serviceClient<wootion_msgs::ControlService>(m_sTerraceService);
    m_InfraredClient = m_NodeHandle.serviceClient<wootion_msgs::ControlService>(m_sInfraredService);
    m_ArmClient = m_NodeHandle.serviceClient<wootion_msgs::ControlService>(m_sArmService);
}

Caller::~Caller()
{
    //nothing to do
}

bool Caller::Call(const CommandType &eCommand, const std::string &sInput, std::string &sOutput)
{
    bool bResult = false;

    switch (eCommand)
    {
        //terrace control
        case pan_left:
        case pan_right:
        case tilt_up:
        case tilt_down:
        case terrace_stop:
        case get_angle:
        case set_angle:
        case get_pan_angle:
        case set_pan_angle:
        case get_tilt_angle:
        case set_tilt_angle:
        case light_on:
        case light_off:
        case brush_on:
        case brush_off:
        {
            wootion_msgs::ControlService TerraceService;
            TerraceService.request.header.stamp = ros::Time::now();
            TerraceService.request.receiver = "terrace_control";
            TerraceService.request.cmd = vsCommand[eCommand];
            bResult = m_TerraceClient.call(TerraceService);
            sOutput = TerraceService.response.data;
            if (!bResult || TerraceService.response.ack == "failed")
            {
                ROS_ERROR("%s call service failed, input:%s, sOutput:%s", vsCommand[eCommand].c_str(), sInput.c_str(), sOutput.c_str());
                return false;
            }
            break;
        }
        //camera control
        case zoom_in:
        case zoom_out:
        case focus_near:
        case focus_far:
        case camera_stop:
        case get_zoom_level:
        case set_zoom_level:
        case set_focus_mode:
        case get_focus_mode:
        case capture_picture:
        case set_osd_mode:
        case get_focus_position:
        case set_focus_position:
        case get_min_focus_distance:
        case set_min_focus_distance:
        case get_wdr_level:
        case set_wdr_level:
        case get_wdr_mode:
        case set_wdr_mode:
        case get_exposure_mode:
        case set_exposure_mode:
        case get_iris:
        case set_iris:
        {
            wootion_msgs::ControlService CameraService;
            CameraService.request.header.stamp = ros::Time::now();
            CameraService.request.receiver = "camera_control";
            CameraService.request.cmd = vsCommand[eCommand];
            CameraService.request.data = sInput;
            bResult = m_CameraClient.call(CameraService);
            sOutput = CameraService.response.data;
            if (!bResult || CameraService.response.ack == "failed")
            {
                ROS_ERROR("%s call service failed, input:%s, sOutput:%s", vsCommand[eCommand].c_str(), sInput.c_str(), sOutput.c_str());
                return false;
            }
            break;
        }
        //infrared imager control
        case infrared_capture_picture:
        case save_temperature:
        {
            wootion_msgs::ControlService InfraredService;
            InfraredService.request.header.stamp = ros::Time::now();
            InfraredService.request.receiver = "infrared_control";
            InfraredService.request.cmd = vsCommand[eCommand];
            InfraredService.request.data = sInput;
            bResult = m_InfraredClient.call(InfraredService);
            sOutput = InfraredService.response.data;
            if (!bResult || InfraredService.response.ack == "failed")
            {
                ROS_ERROR("%s call service failed, input:%s, sOutput:%s", vsCommand[eCommand].c_str(), sInput.c_str(), sOutput.c_str());
                return false;
            }
            break;
        }
        //arm control
        case record_orbit:
        case stop_orbit:
        case play_orbit:
        case rotate:
        case stop_rotate:
        case move:
        case stop_move:
        case get_position:
        case set_position:
        case get_orientation:
        case set_orientation:
        case reset_arm:
        {
            wootion_msgs::ControlService ArmService;
            ArmService.request.header.stamp = ros::Time::now();
            ArmService.request.receiver = "arm_control";
            ArmService.request.cmd = vsCommand[eCommand];
            ArmService.request.data = sInput;
            bResult = m_ArmClient.call(ArmService);
            sOutput = ArmService.response.data;
            if (!bResult || ArmService.response.ack == "failed")
            {
                ROS_ERROR("%s call service failed, input:%s, sOutput:%s", vsCommand[eCommand].c_str(), sInput.c_str(), sOutput.c_str());
                return false;
            }
            break;
        }
        default:
            ROS_ERROR("unknown command:%s, input:%s", vsCommand[eCommand].c_str(), sInput.c_str());
            return false;
    }

    return true;
}

}
