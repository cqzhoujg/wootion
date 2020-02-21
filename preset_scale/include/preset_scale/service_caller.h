//
// Created by LaiHan on 19-12-17.
//

#ifndef SERVICE_CALLER_H
#define SERVICE_CALLER_H

#include "ros/ros.h"
#include "wootion_msgs/ControlService.h"

namespace ServiceCaller
{

typedef enum _CommandType_
{
    //cloud terrace
    pan_left,
    pan_right,
    tilt_up,
    tilt_down,
    terrace_stop,
    get_angle,
    set_angle,
    get_pan_angle,
    get_tilt_angle,
    set_pan_angle,
    set_tilt_angle,
    light_on,
    light_off,
    brush_on,
    brush_off,

    //camera node
    zoom_in,
    zoom_out,
    focus_near,
    focus_far,
    camera_stop,
    get_zoom_level,
    set_zoom_level,
    get_focus_mode,
    set_focus_mode,
    capture_picture,
    set_osd_mode,
    get_focus_position,
    set_focus_position,
    get_min_focus_distance,
    set_min_focus_distance,
    get_wdr_level,
    set_wdr_level,
    get_wdr_mode,
    set_wdr_mode,
    get_exposure_mode,
    set_exposure_mode,
    get_iris,
    set_iris,

    //infrared imager
    infrared_capture_picture,
    save_temperature,

    //robot arm
    record_orbit,
    stop_orbit,
    play_orbit,
    rotate,
    stop_rotate,
    move,
    stop_move,
    get_position,
    set_position,
    get_orientation,
    set_orientation,
    reset_arm,
}CommandType;

const std::vector<std::string> vsCommand =
{
    //cloud terrace
    "pan_left",
    "pan_right",
    "tilt_up",
    "tilt_down",
    "stop",
    "get_angle",
    "set_angle",
    "get_pan_angle",
    "get_tilt_angle",
    "set_pan_angle",
    "set_tilt_angle",
    "light_on",
    "light_off",
    "brush_on",
    "brush_off",

    //camera node
    "zoom_in",
    "zoom_out",
    "focus_near",
    "focus_far",
    "stop",
    "get_zoom_level",
    "set_zoom_level",
    "get_focus_mode",
    "set_focus_mode",
    "capture_picture",
    "set_osd_mode",
    "get_focus_position",
    "set_focus_position",
    "get_min_focus_distance",
    "set_min_focus_distance",
    "get_wdr_level",
    "set_wdr_level",
    "get_wdr_mode",
    "set_wdr_mode",
    "get_exposure_mode",
    "set_exposure_mode",
    "get_iris",
    "set_iris",

    //infrared imager
    "capture_picture",
    "save_temperature",

    //robot arm
    "record_orbit",
    "stop_orbit",
    "play_orbit",
    "rotate",
    "stop_roatate",
    "move",
    "stop_move",
    "get_position",
    "set_position",
    "get_orientation",
    "set_orientation",
    "reset_arm",
};

class Caller
{
public:
    Caller(std::vector<std::string> vsServiceName = {"camera_control", "terrace_control", "infrared_control", "arm_control"});
    ~Caller();
    bool Call(const CommandType &eCommand, const std::string &sInput, std::string &sOutput);
private:
    std::string m_sCameraService;
    std::string m_sTerraceService;
    std::string m_sInfraredService;
    std::string m_sArmService;
    ros::NodeHandle m_NodeHandle;
    ros::ServiceClient m_CameraClient;
    ros::ServiceClient m_TerraceClient;
    ros::ServiceClient m_InfraredClient;
    ros::ServiceClient m_ArmClient;
};

}

#endif //SERVICE_CALLER_H
