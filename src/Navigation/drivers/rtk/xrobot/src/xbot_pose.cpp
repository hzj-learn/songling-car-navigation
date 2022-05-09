/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-05-24 16:58:26 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-06-14 14:08:16
 */

#include "../include/xbot_pose.h"

using namespace Xbot;

bool XbotPose::setup(ros::NodeHandle &n, ros::NodeHandle &private_node)
{
    std::string rtk_port;
    int rtk_baudrate = 115200;
    int iParam;

    private_node.param<std::string>("rtk_port", rtk_port, "xrobot_serial");

    ROS_INFO("Set rtk port:%s ", rtk_port.c_str());

    if (private_node.getParam("rtk_baudrate", rtk_baudrate))
    {
        if (rtk_baudrate != 115200 && rtk_baudrate != 256000)
        {
            ROS_ERROR("Invalid rtk serial baudrate parameter:%d", rtk_baudrate);
        }
        else
        {
            ROS_INFO("Set rtk serial baudrate: %d", rtk_baudrate);
        }
    }

    if (private_node.getParam("debug", iParam))
    {
        if (iParam != 0 && iParam != 1)
        {
            ROS_ERROR("Invalid debug flag parameter:%d", iParam);
        }
        else
        {
            _rtkPose.set_debug_flag(iParam);
            ROS_INFO("Set debug flag: %d", iParam);
        }
    }

    _pubRtkPose = n.advertise<nav_msgs::Odometry>("odom", 100);
    _pubRtkPath = n.advertise<nav_msgs::Path>("path", 100);
    _pubRtkRaw = n.advertise<xrobot::rtk>("rtk", 100);

    _rtkPose.initialize_serial_port(rtk_port, rtk_baudrate);

    return true;
}

void XbotPose::run(ros::NodeHandle &private_node)
{
    _rtkPose.run();
    publish_rtk_msg(_pubRtkPose, _rtkPose.odom_data());
    publish_rtk_msg(_pubRtkPath, _rtkPose.path_data());
    publish_rtk_msg(_pubRtkRaw, _rtkPose.raw_data());
}

