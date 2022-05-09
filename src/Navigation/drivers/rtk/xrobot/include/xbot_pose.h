/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-05-24 16:32:14 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-05-27 16:47:23
 */

#ifndef _XBOT_XBOT_POSE_H_
#define _XBOT_XBOT_POSE_H_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rtk_pose.h"

namespace Xbot
{
class XbotPose
{
public:

    bool setup(ros::NodeHandle &n, ros::NodeHandle &privateNode);
    void run(ros::NodeHandle &privateNode);

    template <class T>
    inline void publish_rtk_msg(ros::Publisher &publisher, const T msg)
    {
        publisher.publish(msg);
    }

private:
    ros::Publisher _pubRtkPose;
    ros::Publisher _pubRtkPath;
    ros::Publisher _pubRtkRaw;
    RtkPose _rtkPose;

    ros::Time _get_data_time;
};
} // namespace Xbot

#endif
