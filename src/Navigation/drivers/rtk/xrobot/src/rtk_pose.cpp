/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-05-27 14:27:35 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-06-13 16:01:16
 */

#include "../include/rtk_pose.h"

using namespace Xbot;

RtkPose::RtkPose()
{
    _rtk_tf.header.frame_id = "odom";
    _rtk_tf.child_frame_id = "rtk";

    _originPose.pose.position.x = 0;
    _originPose.pose.position.y = 0;
    _originPose.pose.position.z = 0;

    _xrobot_raw_rtk.fixmode = 0;
    _xrobot_raw_rtk.nsv = 0;
    _xrobot_raw_rtk.longitude = 0;
    _xrobot_raw_rtk.latitude = 0;
    _xrobot_raw_rtk.altitude = 0;
    _xrobot_raw_rtk.forwardv = 0;
    _xrobot_raw_rtk.north = 0;
    _xrobot_raw_rtk.east = 0;
    _xrobot_raw_rtk.down = 0;
    _xrobot_raw_rtk.headingAngle = 0;
    _xrobot_raw_rtk.roll = 0;
    _xrobot_raw_rtk.pitch = 0;
    _xrobot_raw_rtk.yaw = 0;
    _xrobot_raw_rtk.rollRate = 0;
    _xrobot_raw_rtk.pitchRate = 0;
    _xrobot_raw_rtk.yawRate = 0;

    _num = 0;
    _rtk_fixed_flag = false;
    _message_display_flag = true;
    _debug_flag = false;
    _init_yaw_flag = false;
    _yaw_compensate = 0.0;
}

void RtkPose::initialize_serial_port(const std::string serial_port, const int serial_baudrate)
{

    try
    {
        _rtkPort.setPort(serial_port);
        _rtkPort.setBaudrate(serial_baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        _rtkPort.setTimeout(to);
        _rtkPort.open();
    }
    catch (serial::IOException &er)
    {
        ROS_INFO("Unable to open rtk port !!! ");
    }

    if (_rtkPort.isOpen())
    {
        ROS_INFO("Serial port rkt initialized!!!");
    }
}

void RtkPose::run()
{
    get_rtk_data();
    pub_raw_data();
    if (_xrobot_raw_rtk.fixmode == 6 || _debug_flag == true)
    {
        if (!_rtk_fixed_flag)
        {
            ROS_INFO_STREAM("Rtk fixed,number of satellites is: " << _xrobot_raw_rtk.nsv);

            _rtk_fixed_flag = true;
        }
        pose_data_transform();
        yaw_compensate();
        pub_path_data();
        pub_tf_data();
        pub_odom_data();
    }
    else
    {
        if (_message_display_flag)
        {
            ROS_INFO_STREAM("No enter RTK,current mode is: " << _xrobot_raw_rtk.fixmode);
            ROS_INFO("Received xrobot rtk info:fixmode,%d,nsv,%d,lon,%d,lat,%d,alt,%d,forv,%d,north,%d,east,%d,down,%d,headA,%d,roll,%d,pitch,%d,yaw,%d,rollRate,%d,pitchRate,%d,yawRate,%d",
                     _xrobot_raw_rtk.fixmode,
                     _xrobot_raw_rtk.nsv,
                     _xrobot_raw_rtk.longitude,
                     _xrobot_raw_rtk.latitude,
                     _xrobot_raw_rtk.altitude,
                     _xrobot_raw_rtk.forwardv,
                     _xrobot_raw_rtk.north,
                     _xrobot_raw_rtk.east,
                     _xrobot_raw_rtk.down,
                     _xrobot_raw_rtk.headingAngle,
                     _xrobot_raw_rtk.roll,
                     _xrobot_raw_rtk.pitch,
                     _xrobot_raw_rtk.yaw,
                     _xrobot_raw_rtk.rollRate,
                     _xrobot_raw_rtk.pitchRate,
                     _xrobot_raw_rtk.yawRate);
            _message_display_flag = false;
        }

        _rtk_fixed_flag = false;
    }

    if (_num == 99)
    {
        _message_display_flag = true;
        _num = 0;
    }

    _num++;
}
/*
void RtkPose::get_rtk_data(const xrobot::rtkConstPtr rtk)
{
    _get_data_time = rtk->header.stamp;
    _xrobot_raw_rtk.fixmode = rtk->fixmode;
    _xrobot_raw_rtk.nsv = rtk->nsv;
    _xrobot_raw_rtk.longitude = rtk->longitude * pow(10, 7);
    _xrobot_raw_rtk.latitude = rtk->latitude * pow(10, 7);
    _xrobot_raw_rtk.altitude = rtk->altitude * 1000;
    _xrobot_raw_rtk.forwardv = rtk->forwardv * 100;
    _xrobot_raw_rtk.north = rtk->north * 100;
    _xrobot_raw_rtk.east = rtk->east * 100;
    _xrobot_raw_rtk.down = rtk->down * 100;
    _xrobot_raw_rtk.headingAngle = rtk->headingAngle * 10;
    _xrobot_raw_rtk.roll = rtk->roll * 573;
    _xrobot_raw_rtk.pitch = rtk->pitch * 573;
    _xrobot_raw_rtk.yaw = rtk->yaw * 573;
    _xrobot_raw_rtk.rollRate = rtk->rollRate * 573;
    _xrobot_raw_rtk.pitchRate = rtk->pitchRate * 573;
    _xrobot_raw_rtk.yawRate = rtk->yawRate * 573;
}
*/
void RtkPose::get_rtk_data()
{
    uint8_t data = 0; //数据buffer

    static uint8_t data_fromrtk[66] = {0};
    static int receive_counter = 0;
    static int dispaly_counter = 0;
    int len = 0;

    int data_size = 0;

    unsigned char start = 0x23; //起始位 0x23 #
    //unsigned char checksum = 0x00; // 初始化

    data_size = sizeof(xrobot_Raw_rtk_t);
    //ROS_DEBUG("data_size:%d ", data_size);

    len = _rtkPort.available();

    while (len > 0)
    {
        len--;
        _rtkPort.read(&data, 1);
        ROS_DEBUG_STREAM("data: " << int(data));

        data_fromrtk[receive_counter] = data;

        // 给第一个赋值
        /* if (receive_counter == 0)
        {
            data_fromrtk[receive_counter] = data;
        }*/

receive_counter++;

        if ((receive_counter == 1) && (data_fromrtk[0] != start))
        {
            ROS_DEBUG_STREAM("Rtk data start is not 0x23,receive_counter: " << receive_counter << " data_fromchassis: " << data_fromrtk[0]);
            receive_counter = 0;
        }

        // 计算校验码
        if (receive_counter == data_size + 2)
        {
            ROS_DEBUG_STREAM("Raw checksum: " << int(data));
            unsigned char checksum = clac_checksum(data_fromrtk);
            ROS_DEBUG_STREAM("Rtk checksum: " << int(checksum));
            if (data != checksum)
            {
                ROS_DEBUG_STREAM("Rtk checksum: " << int(checksum) << " is not match!");
                receive_counter = 0;
            }
        }

        if (receive_counter >= data_size + 2)
        {
            receive_counter = 0;
            memcpy(&_xrobot_raw_rtk, data_fromrtk + 1, data_size);
            _get_data_time = ros::Time::now();
            dispaly_counter++;

            if (dispaly_counter > 20)
            {
                ROS_DEBUG("Received xrobot rtk info:fixmode,%d,nsv,%d,lon,%d,lat,%d,alt,%d,forv,%d,north,%d,east,%d,down,%d,headA,%d,roll,%d,pitch,%d,yaw,%d,rollRate,%d,pitchRate,%d,yawRate,%d",
                          _xrobot_raw_rtk.fixmode,
                          _xrobot_raw_rtk.nsv,
                          _xrobot_raw_rtk.longitude,
                          _xrobot_raw_rtk.latitude,
                          _xrobot_raw_rtk.altitude,
                          _xrobot_raw_rtk.forwardv,
                          _xrobot_raw_rtk.north,
                          _xrobot_raw_rtk.east,
                          _xrobot_raw_rtk.down,
                          _xrobot_raw_rtk.headingAngle,
                          _xrobot_raw_rtk.roll,
                          _xrobot_raw_rtk.pitch,
                          _xrobot_raw_rtk.yaw,
                          _xrobot_raw_rtk.rollRate,
                          _xrobot_raw_rtk.pitchRate,
                          _xrobot_raw_rtk.yawRate);
                dispaly_counter = 0;
            }
        }
    }
}

unsigned char RtkPose::clac_checksum(uint8_t *data)
{
    uint8_t sum = 0;
    unsigned char checksum;

    for (int i = 1; i < 65; i++)
    {
        sum += data[i];
        ROS_DEBUG_STREAM("sum_data:" << int(data[i]));
        ROS_DEBUG_STREAM("sum:" << int(sum));
    }

    checksum = (unsigned char)sum;

    return checksum;
}

void RtkPose::pub_raw_data()
{
    _rtk_raw.header.frame_id = "rtk";
    _rtk_raw.header.stamp = _get_data_time;
    _rtk_raw.fixmode = _xrobot_raw_rtk.fixmode;
    _rtk_raw.nsv = _xrobot_raw_rtk.nsv;
    _rtk_raw.longitude = _xrobot_raw_rtk.longitude / pow(10, 7);
    _rtk_raw.latitude = _xrobot_raw_rtk.latitude / pow(10, 7);
    _rtk_raw.altitude = _xrobot_raw_rtk.altitude / 1000.0;
    _rtk_raw.forwardv = _xrobot_raw_rtk.forwardv / 100.0;
    _rtk_raw.north = _xrobot_raw_rtk.north / 100.0f;
    _rtk_raw.east = _xrobot_raw_rtk.east / 100.0f;
    _rtk_raw.down = _xrobot_raw_rtk.down / 100.0f;
    _rtk_raw.headingAngle = _xrobot_raw_rtk.headingAngle / 10.0f;
    _rtk_raw.roll = _xrobot_raw_rtk.roll / 57.3 / 10.0f;
    _rtk_raw.pitch = _xrobot_raw_rtk.pitch / 57.3 / 10.0f;
    _rtk_raw.yaw = _xrobot_raw_rtk.yaw / 57.3 / 10.0f;
    _rtk_raw.rollRate = _xrobot_raw_rtk.rollRate / 57.3 / 10.0f;
    _rtk_raw.pitchRate = _xrobot_raw_rtk.pitchRate / 57.3 / 10.0f;
    _rtk_raw.yawRate = _xrobot_raw_rtk.yawRate / 57.3 / 10.0f;
}

void RtkPose::pose_data_transform()
{
    _rawPose.pose.position.x = _xrobot_raw_rtk.latitude;
    _rawPose.pose.position.y = _xrobot_raw_rtk.longitude;
    _rawPose.pose.position.z = _xrobot_raw_rtk.altitude;

    // 转换LLS到NED
    ROS_DEBUG("originPose.x:%lf,originPose.y:%lf,originPose.z:%lf,\n,xrobot_raw_rtk.latitude.x:%d,xrobot_raw_rtk.longitude:%d,xrobot_raw_rtk.altitude:%d",
              _originPose.pose.position.x,
              _originPose.pose.position.y,
              _originPose.pose.position.z,
              _xrobot_raw_rtk.latitude,
              _xrobot_raw_rtk.longitude,
              _xrobot_raw_rtk.altitude);
    // 设置起始点及航向补偿
    if (_rawPose.pose.position.x >= EPS && _originPose.pose.position.x <= EPS)
    {
        _originPose.pose.position.x = _rawPose.pose.position.x;
        _originPose.pose.position.y = _rawPose.pose.position.y;
        _originPose.pose.position.z = _rawPose.pose.position.z;
    }

    _con.setLls(_rawPose);
    _con.setLlsTakeOff(_originPose);
    _con.llsToNed();

    // 转换精度
    Eigen::Vector3d ned;

    // xy坐标系上的值
    ned = _con.getNed();
    _xrobot_odom.x = precisionFilter(ned[0], 3);
    _xrobot_odom.y = precisionFilter(ned[1], 3);
    _xrobot_odom.z = ned[2] / 1000; // 高度为mm

    // xyz速度
    _xrobot_odom.vx = float(_xrobot_raw_rtk.north) / 100.0;
    _xrobot_odom.vy = float(_xrobot_raw_rtk.east) / 100.0;
    _xrobot_odom.vz = float(_xrobot_raw_rtk.down) / 100.0;

    //角度值 原始为degree 实际中需要radius
    _xrobot_odom.roll = float(_xrobot_raw_rtk.roll) / 57.3 / 10;
    _xrobot_odom.pitch = float(_xrobot_raw_rtk.pitch) / 57.3 / 10;
    int num;
    num = _xrobot_raw_rtk.yaw / 3600;
    _xrobot_odom.yaw = float(_xrobot_raw_rtk.yaw) / 57.3 / 10 - 2 * M_PI * num;
    ROS_DEBUG_STREAM("raw_yaw: " << _xrobot_raw_rtk.yaw);
    ROS_DEBUG_STREAM("yaw: " << _xrobot_odom.yaw);

    //角速度
    _xrobot_odom.rollRate = _xrobot_raw_rtk.rollRate / 57.3 / 10;
    _xrobot_odom.pitchRate = _xrobot_raw_rtk.pitchRate / 57.3 / 10;
    _xrobot_odom.yawRate = _xrobot_raw_rtk.yawRate / 57.3 / 10;

    static int display_counter1 = 0;
    display_counter1++;
    if (display_counter1 > 20)
    {
        ROS_DEBUG("xrobot_odom.x:%f,xrobot_odom.y:%f,xrobot_odom.z:%f,xrobot_odom.roll:%f,xrobot_odom.pitch:%f,xrobot_odom.yaw:%f,xrobot_odom.rollRata:%f,xrobot_odom.pitchRate:%f,xrobot_odom.yawRate:%f",
                  _xrobot_odom.x,
                  _xrobot_odom.y,
                  _xrobot_odom.z,
                  _xrobot_odom.roll,
                  _xrobot_odom.pitch,
                  _xrobot_odom.yaw,
                  _xrobot_odom.rollRate,
                  _xrobot_odom.pitchRate,
                  _xrobot_odom.yawRate);
        display_counter1 = 0;
    }
}

double RtkPose::precisionFilter(double data, int n)
{
    int mid;
    mid = data * (pow10(n));
    return mid / pow10(n);
}

void RtkPose::yaw_compensate()
{
    if (!_init_yaw_flag)
    {
        //初始化欧拉角 yaw pitch roll
        Eigen::Vector3d eulerAngle(_xrobot_odom.yaw, _xrobot_odom.pitch, _xrobot_odom.roll);

        //欧拉角转四元数
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));

        /*由NED更改到ROS坐标系(NEU)
          需要绕Z轴旋转-90度
          绕x轴旋转180度*/
        Eigen::AngleAxisd rotZ(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
        //Eigen::AngleAxisd rotY(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotX(1.0 * M_PI, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond quaternion;
        quaternion = rotX * rotZ * yawAngle * pitchAngle * rollAngle;

        Eigen::Vector3d befEulerAngle = quaternion.toRotationMatrix().eulerAngles(2,1,0);
        ROS_DEBUG_STREAM("befEulerAngle:"<<befEulerAngle);
        _yaw_compensate = -befEulerAngle(0);

        _init_yaw_flag = true;
    }
}

void RtkPose::pub_path_data()
{
    geometry_msgs::PoseStamped currentPose;
    currentPose.header.frame_id = "odom";
    currentPose.header.stamp = _get_data_time;
    currentPose.pose.position.x = _xrobot_odom.y * cos(_yaw_compensate) - _xrobot_odom.x * sin(_yaw_compensate);
    currentPose.pose.position.y = _xrobot_odom.y * sin(_yaw_compensate) + _xrobot_odom.x * cos(_yaw_compensate);
    currentPose.pose.position.z = -_xrobot_odom.z;

    _rtk_path.header = currentPose.header;
    _rtk_path.poses.push_back(currentPose);
}

void RtkPose::pub_tf_data()
{

    /* 由NED更改到ROS坐标系(NEU)
       需要绕x轴旋转180度
       等价于xy互换，z相反*/
    _rtk_tf.transform.translation.x = _xrobot_odom.y * cos(_yaw_compensate) - _xrobot_odom.x * sin(_yaw_compensate);
    _rtk_tf.transform.translation.y = _xrobot_odom.y * sin(_yaw_compensate) + _xrobot_odom.x * cos(_yaw_compensate);
    _rtk_tf.transform.translation.z = -_xrobot_odom.z;

    //初始化欧拉角 yaw pitch roll
    Eigen::Vector3d eulerAngle(_xrobot_odom.yaw, _xrobot_odom.pitch, _xrobot_odom.roll);

    //欧拉角转四元数
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));

    /*由NED更改到ROS坐标系(NEU)
    -需要绕Z轴旋转-90度-
    绕x轴旋转180度*/
    Eigen::AngleAxisd rotZ(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd rotY(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotX(1.0 * M_PI, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotZc(_yaw_compensate, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond quaternion;
    quaternion = rotZc * rotX * rotZ * yawAngle * pitchAngle * rollAngle;

    _rtk_tf.transform.rotation.x = quaternion.x();
    _rtk_tf.transform.rotation.y = quaternion.y();
    _rtk_tf.transform.rotation.z = quaternion.z();
    _rtk_tf.transform.rotation.w = quaternion.w();

    _rtk_tf.header.stamp = _get_data_time;

    _rtk_broadcaster.sendTransform(_rtk_tf);
}

void RtkPose::pub_odom_data()
{
    _rtk_odom.header.stamp = _get_data_time;

    _rtk_odom.header.frame_id = "odom";

    /* 由NED更改到ROS坐标系(NEU)
       -需要绕Z轴旋转-90度-
       绕X轴旋转180度*/
    _rtk_odom.pose.pose.position.x = _xrobot_odom.y * cos(_yaw_compensate) - _xrobot_odom.x * sin(_yaw_compensate);
    _rtk_odom.pose.pose.position.y = _xrobot_odom.y * sin(_yaw_compensate) + _xrobot_odom.x * cos(_yaw_compensate);
    _rtk_odom.pose.pose.position.z = -_xrobot_odom.z;
    ROS_DEBUG_STREAM("_xrobot_odom.x: " << _xrobot_odom.x << " _xrobot_odom.y: " << _xrobot_odom.y << " _xrobot_odom.z: " << _xrobot_odom.z);

    //将四元数转换欧拉角
    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (_rtk_tf.transform.rotation.w * _rtk_tf.transform.rotation.z + _rtk_tf.transform.rotation.x * _rtk_tf.transform.rotation.y);
    double cosy_cosp = +1.0 - 2.0 * (_rtk_tf.transform.rotation.y * _rtk_tf.transform.rotation.y + _rtk_tf.transform.rotation.z * _rtk_tf.transform.rotation.z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    ROS_INFO("%lf",yaw);

    double pitch = 0;
    double roll = 0;
    Eigen::Vector3d Ang(yaw,pitch,roll);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(Ang[0], ::Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(Ang[1], ::Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(Ang[2], ::Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q;
    q = R;

    _rtk_odom.pose.pose.orientation.x=q.x();
    _rtk_odom.pose.pose.orientation.y=q.y();
    _rtk_odom.pose.pose.orientation.z=q.z();
    _rtk_odom.pose.pose.orientation.w=q.w();
   
    //_rtk_odom.pose.pose.orientation = _rtk_tf.transform.rotation; //引用之前的

    /* 由NED更改到ROS坐标系(NEU)
       需要绕Z轴旋转-90度
       绕X轴旋转180度*/
    _rtk_odom.twist.twist.linear.x = _xrobot_odom.vy;
    _rtk_odom.twist.twist.linear.y = _xrobot_odom.vx;
    _rtk_odom.twist.twist.linear.z = 0;
    _rtk_odom.twist.twist.angular.x = 0;
    _rtk_odom.twist.twist.angular.y = 0;
    _rtk_odom.twist.twist.angular.z = -_xrobot_odom.yawRate;
}
