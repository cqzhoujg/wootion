/*
 * robot_control_node.cpp
 *
 *  Created on: Dec 06, 2019
 *      Author: ZhouJinGang
 */
#include "CEliteControl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_arm");
    CEliteControl elite_control;

    ros::MultiThreadedSpinner spinner(6); // Use 6 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return 0;
}
