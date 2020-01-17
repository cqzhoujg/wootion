/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2018-12-25
Description: main
**************************************************/

#include <iostream>
#include <string>
#include <ctime>
#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include "CDeviceNetMaster.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "devicenet_master");

    CDeviceNetMaster deviceNet_master;

    ros::spin();

    return 0;
}
