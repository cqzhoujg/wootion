//
// Created by ros on 18-12-21.
//

#include "preset_scale.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "preset_scale");

    ros::Time::init();

    PresetScale::PresetScale preset_scale;

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return 0;
}