/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-05-24 16:49:36 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-05-27 14:54:53
 */
#include "../include/xbot_pose.h"

using namespace Xbot;

int main(int argv, char **argc)
{
    ros::init(argv, argc, "xbot_pose");

    ros::NodeHandle n;
    ros::NodeHandle private_node("~");

    XbotPose xbotPose;

    ros::Rate Rate(50);

    xbotPose.setup(n, private_node);

    while (ros::ok())
    {
        xbotPose.run(private_node);
        ros::spinOnce();
        Rate.sleep();
    }

    return 0;
}
