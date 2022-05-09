#include "../include/Converter.h"

using namespace Xbot;

Converter::Converter()
{
    _lls = Eigen::Vector3d::Zero();
    _lls_takeoff = Eigen::Vector3d::Zero();
    _ned = Eigen::Vector3d::Zero();
}

void Converter::setLls(const geometry_msgs::PoseStamped &rawPose)
{
    _lls[0] = rawPose.pose.position.x;
    _lls[1] = rawPose.pose.position.y;
    _lls[2] = rawPose.pose.position.z;
}

void Converter::setLlsTakeOff(const geometry_msgs::PoseStamped &originPose)
{
    _lls_takeoff[0] = originPose.pose.position.x;
    _lls_takeoff[1] = originPose.pose.position.y;
    _lls_takeoff[2] = originPose.pose.position.z;
}

Eigen::Vector3d Converter::llsToNed()
{
    // 内存不足
    double temp = static_cast<double>(_lls[1] - _lls_takeoff[1]) * LANLON_COEFFICIENT;
    temp *= cos(static_cast<double>(_lls[0]) / pow(10, 7) * 0.0174532925f);
    _ned[1] = temp;
    temp = static_cast<double>(_lls[0] - _lls_takeoff[0]) * LANLON_COEFFICIENT;
    _ned[0] = temp;
    _ned[2] = _lls[2] - _lls_takeoff[2];
    return _ned;
}
/*
void Converter::nedToLls()
{
    _lls[0] = static_cast<int32_t>(static_cast<double>(_ned[0]) / LANLON_COEFFICIENT) + _lls_takeoff[0];
    _lls[1] = static_cast<int32_t>((static_cast<double>(_ned[1]) / cos(static_cast<double>(_lls[0]) / pow(10, 7) * 0.0174532925f)) / LANLON_COEFFICIENT) + _lls_takeoff[1];
    _lls[2] = _ned[2];
}*/
