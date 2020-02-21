
#include "find_scale.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_scale");

    ros::Time::init();

    FindScale::FindScale find_scale;

    ros::spin();

    return 0;
}