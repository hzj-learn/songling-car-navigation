/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-05-24 16:20:26 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-05-29 15:32:22
 */

#ifndef _XBOT_RTK_POSE_H_
#define _XBOT_RTK_POSE_H_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include "Converter.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>
#include "xrobot/rtk.h"

#define EPS 0.0000001

// 初始坐标
static geometry_msgs::PoseStamped _originPose;

namespace Xbot
{
class RtkPose
{
public:
    typedef struct
    {
        int32_t fixmode;   // 状态 6 锁定
        int32_t nsv;       // 卫星数
        int32_t longitude; //7位
        int32_t latitude;
        int32_t altitude;
        int32_t forwardv;     //前向速度 cm/s
        int32_t north;        //北向速度
        int32_t east;         //东向速度
        int32_t down;         //向地速度
        int32_t headingAngle; //前向角度 0.1degree
        int32_t roll;
        int32_t pitch;
        int32_t yaw;
        int32_t rollRate; //角速度 0.1degree/s
        int32_t pitchRate;
        int32_t yawRate;
    } xrobot_Raw_rtk_t;

    typedef struct
    {
        float x;
        float y;
        float z;
        float vx;
        float vy;
        float vz;
        float roll;
        float pitch;
        float yaw;
        float rollRate;
        float pitchRate;
        float yawRate;
    } xrobot_Odom_t;

    RtkPose();

    void initialize_serial_port(const std::string serial_port, const int serial_baudrate);

    void run();

    // 从串口获取rtk数据
    void get_rtk_data();

    unsigned char clac_checksum(uint8_t *data);

    // 发布原始数据
    void pub_raw_data();

    // 转换姿态数据
    void pose_data_transform();

    void yaw_compensate();

    double precisionFilter(double data, int n);

    void pub_path_data();

    void pub_tf_data();

    void pub_odom_data();

    inline nav_msgs::Odometry odom_data() { return _rtk_odom; }
    inline nav_msgs::Path path_data() { return _rtk_path; }
    inline xrobot::rtk raw_data() { return _rtk_raw; }
    inline void set_debug_flag(bool debug_flag) { _debug_flag = debug_flag; }

private:
    //rtk串口
    serial::Serial _rtkPort;

    // debug标志
    bool _debug_flag;

    geometry_msgs::TransformStamped _rtk_tf;

    tf2_ros::TransformBroadcaster _rtk_broadcaster;

    xrobot_Raw_rtk_t _xrobot_raw_rtk;

    xrobot_Odom_t _xrobot_odom;

    // 转换坐标系
    Converter _con;

    geometry_msgs::PoseStamped _rawPose;

    ros::Time _get_data_time;

    nav_msgs::Odometry _rtk_odom;

    nav_msgs::Path _rtk_path;

    xrobot::rtk _rtk_raw;

    bool _rtk_fixed_flag;

    bool _message_display_flag;

    int _num;

    float _yaw_compensate;

    bool _init_yaw_flag;
};
} // namespace Xbot

#endif
