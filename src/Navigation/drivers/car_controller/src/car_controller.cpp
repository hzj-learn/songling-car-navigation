#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include <iostream>

#include "ros/ros.h"
#include <sstream>
#include <geometry_msgs/Twist.h>

#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ctime>
#include <stdint.h>

#include "autoware_msgs/VehicleStatus.h"//自定义消息类型

#define PI 3.141592653

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。

using namespace std;

double vel_x = 0.0;//小车的线速度
double vel_th = 0.0;//小车的角速度

int speed_step = 0;//下发的速度，单位:mm/s
int angle_step = 19;//下发的角度，也就是can协议里面的数据N，初始角度为0时候数据N为19

short back_wheel_speed = 0.0;//上报的后轮速度 mm/s
double back_wheel_convert_speed = 0.0;//转换后后轮的速度 m/s
short turn_angle = 0.0;//上报的转向角度
int battery_level = 0.0;//上报的电量百分比
int error_flag = 0.0;//上报的错误状态

void *receive_func(void* param)  //接收线程。
{
    int reclen=0;
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i,j;

    int *run=(int*)param;//线程启动，退出控制。
    int ind=0;

    while((*run)&0x0f)
    {
        if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            j=0;
            {
                if(rec[j].ID==0x02)
                {
                    //printf("Index:%04d  ",count);count++;//序号递增
                    printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                    if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                    if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                    if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
                    if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                    printf("DLC:0x%02X",rec[j].DataLen);//帧长度
                    printf(" data:0x");	//数据
                    for(i = 0; i < rec[j].DataLen; i++)
                    {
                        printf(" %02X", rec[j].Data[i]);
                    }
                    printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
                    printf("\n");
                    break;
                }
            }
        }
    }
    printf("run thread exit\n");//退出接收线程
    pthread_exit(0);
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)//速度控制回调
{
    geometry_msgs::Twist twist = twist_aux;

    vel_x = twist_aux.linear.x;
    vel_th = twist_aux.angular.z;

    vel_th = (vel_th * 180) / (33.25 * M_PI);//速度单位转化为弧度值
    //cout << "vel_x: " << vel_x << endl;
    //cout << "vel_th: " << vel_th << endl;

    //下面这些判断根据can协议角度表来定义执行
    if(vel_x > 0)//小车前进
    {
        speed_step = vel_x * 1000;//ros里面的m/s转化为mm/s
	cout << "前进: " << speed_step << " mm/s" << endl;//单位：mm/s
    }

    if(vel_x < 0)//小车后退
    {
        speed_step = vel_x * 1000;//ros里面的m/s转化为mm/s
	cout << "后退: " << speed_step << " mm/s" << endl;//单位：mm/s
    }

    if(vel_th > 0 && vel_th <= 1/18.00)//小车左转，根据协议角度表分为18.00等份
    {
        angle_step = 20;
        cout << "左转 3.50度" << endl;
    }
    if(vel_th > 1/18.00 && vel_th <= 2/18.00)
    {
        angle_step = 21;
        cout << "左转 5.25度" << endl;
    }
    if(vel_th > 2/18.00 && vel_th <= 3/18.00)
    {
        angle_step = 22;
        cout << "左转 7.00度" << endl;
    }
    if(vel_th > 3/18.00 && vel_th <= 4/18.00)
    {
        angle_step = 23;
        cout << "左转 8.75度" << endl;
    }
    if(vel_th > 4/18.00 && vel_th <= 5/18.00)
    {
        angle_step = 24;
        cout << "左转 10.05度" << endl;
    }
    if(vel_th > 5/18.00 && vel_th <= 6/18.00)
    {
        angle_step = 25;
        cout << "左转 12.25度" << endl;
    }
    if(vel_th > 6/18.00 && vel_th <= 7/18.00)
    {
        angle_step = 26;
        cout << "左转 14.00度" << endl;
    }
    if(vel_th > 7/18.00 && vel_th <= 8/18.00)
    {
        angle_step = 27;
        cout << "左转 15.75度" << endl;
    }
    if(vel_th > 8/18.00 && vel_th <= 9/18.00)
    {
        angle_step = 28;
        cout << "左转 17.50度" << endl;
    }
    if(vel_th > 9/18.00 && vel_th <= 10/18.00)
    {
        angle_step = 29;
        cout << "左转 19.25度" << endl;
    }
    if(vel_th > 10/18.00 && vel_th <= 11/18.00)
    {
        angle_step = 30;
        cout << "左转 21.00度" << endl;
    }
    if(vel_th > 11/18.00 && vel_th <= 12/18.00)
    {
        angle_step = 31;
        cout << "左转 22.75度" << endl;
    }
    if(vel_th > 12/18.00 && vel_th <= 13/18.00)
    {
        angle_step = 32;
        cout << "左转 24.50度" << endl;
    }
    if(vel_th > 13/18.00 && vel_th <= 14/18.00)
    {
        angle_step = 33;
        cout << "左转 26.25度" << endl;
    }
    if(vel_th > 14/18.00 && vel_th <= 15/18.00)
    {
        angle_step = 34;
        cout << "左转 28.00度" << endl;
    }
    if(vel_th > 15/18.00 && vel_th <= 16/18.00)
    {
        angle_step = 35;
        cout << "左转 29.75度" << endl;
    }
    if(vel_th > 16/18.00 && vel_th <= 17/18.00)
    {
        angle_step = 36;
        cout << "左转 31.50度" << endl;
    }
    //if(vel_th > 17/18.00 && vel_th <= 1)
    if(vel_th > 17/18.00)
    {
        angle_step = 37;
        cout << "左转 33.25度" << endl;
    }

    if(vel_th < 0 && vel_th >= -1/18.00)//小车右转，根据协议角度表分为18.00等份
    {
        angle_step = 18.00;
        cout << "右转 3.50度" << endl;
    }
    if(vel_th < -1/18.00 && vel_th >= -2/18.00)
    {
        angle_step = 17;
        cout << "右转 5.25度" << endl;
    }
    if(vel_th < -2/18.00 && vel_th >= -3/18.00)
    {
        angle_step = 16;
        cout << "右转 7.00度" << endl;
    }
    if(vel_th < -3/18.00 && vel_th >= -4/18.00)
    {
        angle_step = 15;
        cout << "右转 8.75度" << endl;
    }
    if(vel_th < -4/18.00 && vel_th >= -5/18.00)
    {
        angle_step = 14;
        cout << "右转 10.50度" << endl;
    }
    if(vel_th < -5/18.00 && vel_th >= -6/18.00)
    {
        angle_step = 13;
        cout << "右转 12.25度" << endl;
    }
    if(vel_th < -6/18.00 && vel_th >= -7/18.00)
    {
        angle_step = 12;
        cout << "右转 14.00度" << endl;
    }
    if(vel_th < -7/18.00 && vel_th >= -8/18.00)
    {
        angle_step = 11;
        cout << "右转 15.75度" << endl;
    }
    if(vel_th < -8/18.00 && vel_th >= -9/18.00)
    {
        angle_step = 10;
        cout << "右转 17.50度" << endl;
    }
    if(vel_th < -9/18.00 && vel_th >= -10/18.00)
    {
        angle_step = 9;
        cout << "右转 19.25度" << endl;
    }
    if(vel_th < -10/18.00 && vel_th >= -11/18.00)
    {
        angle_step = 8;
        cout << "右转 21.00度" << endl;
    }
    if(vel_th < -11/18.00 && vel_th >= -12/18.00)
    {
        angle_step = 7;
        cout << "右转 22.75度" << endl;
    }
    if(vel_th < -12/18.00 && vel_th >= -13/18.00)
    {
        angle_step = 6;
        cout << "右转 24.50度" << endl;
    }
    if(vel_th < -13/18.00 && vel_th >= -14/18.00)
    {
        angle_step = 5;
        cout << "右转 26.25度" << endl;
    }
    if(vel_th < -14/18.00 && vel_th >= -15/18.00)
    {
        angle_step = 4;
        cout << "右转 28.00度" << endl;
    }
    if(vel_th < -15/18.00 && vel_th >= -16/18.00)
    {
        angle_step = 3;
        cout << "右转 29.75度" << endl;
    }
    if(vel_th < -16/18.00 && vel_th >= -17/18.00)
    {
        angle_step = 2;
        cout << "右转 31.50度" << endl;
    }
    //if(vel_th < -17/18.00 && vel_th >= -1)
    if(vel_th < -17/18.00)
    {
        angle_step = 1;
        cout << "右转 33.25度" << endl;
    }
    
    if(vel_x == 0 && vel_th == 0)//小车停止
    {
        speed_step = 0;
        angle_step = 19;//如果停止时候不需要打角回0,注释这一行代码
        cout << "停止 档位" << endl;
    }
}

int main(int argc, char **argv)
{
    int i=0;
    printf(">>this is hello !\r\n");//指示程序已运行
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error!\n");
        exit(1);
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;//FFFFFFFF全部接收
    config.Filter=2;//接收所有帧  2-只接受标准帧  3-只接受扩展帧
    config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
    config.Timing1=0x1C;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }
    //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;
    //需要读取的帧，结构体设置
    VCI_CAN_OBJ rev[1];
    rev[0].SendType=0;
    rev[0].RemoteFlag=0;
    rev[0].ExternFlag=0;
    rev[0].DataLen=8;

    int m_run0=1;
    pthread_t threadid;
    int ret;
    //ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
    int times = 5;

    ros::init(argc, argv, "car_controller");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 100, cmd_velCallback);//速度回调

    ros::Publisher car_controller_pub = n.advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 10);//自定义消息发布

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        //要写入的数据
        send[0].Data[0]=0x00000001;
        send[0].Data[1]=1;
        send[0].Data[2]=speed_step & 0xFF;
        send[0].Data[3]=(speed_step>>8) & 0xFF;
        send[0].Data[4]=angle_step & 0xFF;
        send[0].Data[5]=(angle_step>>8) & 0xFF;
        send[0].Data[6]=0x00;
        send[0].Data[7]=0x00;

        //写入数据
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
        {
           //printf("TX data successful!\n");
        }

        autoware_msgs::VehicleStatus msg;

        //读取数据
        if(VCI_Receive(VCI_USBCAN2, 0, 0,rev, 2500, 0)>0)
        {
            if((rev[0].Data[0] == 02) && (rev[0].Data[1] == 02))//can发送02代表接收的数据
            {
                back_wheel_speed = (rev[0].Data[3] << 8) | rev[0].Data[2];//上报的后轮速度
                back_wheel_convert_speed = (static_cast<double>(back_wheel_speed))/1000.0;
                turn_angle = (rev[0].Data[5] << 8) | rev[0].Data[4];//上报的转向角度
                battery_level = rev[0].Data[6];//上报的电量百分比
	        error_flag = rev[0].Data[7];//上报的错误状态
                cout << "后轮速度: " << back_wheel_convert_speed << " m/s" << endl;
	        cout << "转向弧度: " << turn_angle/100.00*M_PI/180 << endl;
	        cout << "错误状态: " << error_flag << endl;	
	        cout << "电量百分比: " << battery_level << "%" << endl;

		msg.speed = back_wheel_convert_speed;
		msg.angle = turn_angle/100.00*M_PI/180;
		msg.battery_level = battery_level;
		msg.error_flag = error_flag;
            }
	}

        car_controller_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。

    return 0;
    //除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
    //goto ext;
}
