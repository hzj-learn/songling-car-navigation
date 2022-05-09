/*
 * @Author: Jian Xu @Xag 
 * @Date: 2018-11-22 15:11:34 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2018-11-22 16:32:25
 */

#ifndef _XBOT_CONVERTER_H_
#define _XBOT_CONVERTER_H_

#include <math.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>

#define LANLON_COEFFICIENT 0.01112f

namespace Xbot
{
class Converter
{
  public:

    Converter();
    
    void setLls(const geometry_msgs::PoseStamped& rawPose);
    void setLlsTakeOff(const geometry_msgs::PoseStamped& originPose);

    Eigen::Vector3d llsToNed();
    Eigen::Vector3d nedToLls();
    
    inline Eigen::Vector3d getNed(){return _ned;}
  private:

    // 经纬度为原始数据
    Eigen::Vector3d _lls;
    Eigen::Vector3d _lls_takeoff;
    Eigen::Vector3d _ned;
};
} // namespace Map

#endif