    //新的避障方案
	/*
	author: Taosit_Su  
	date: 2020-03-30
	因为雷达扫描的是园型，如果用来避障取固定返回距离，是个扇形避障区
	实际上使用是个矩形避障区域，以下代码完成，扇形到矩形避张区域的转换
	只需要修改参数：
	--degreeStart和degreeEnd是设定左边的扇形范围
	--degreeStartII和degreeEndII是设定右边的扇形范围
	--avoid_distances是需要的车身前方避障距离
	--car_width是车身宽度的一半，如果车宽2m，car_width设为1
	*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <stdint.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


#include "controlcan.h"
#include "unistd.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

 
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define PI 3.141592653



VCI_BOARD_INFO pInfo;//用来获取设备信息。
ros::Publisher pub;

int can1 = 0;
int can2 = 1;
using namespace std;
using namespace boost::asio;



float vel_x = 0.0;
float vel_th = 0.0;
float brake = 0.0;
int cmd_start = 0;

float twist_vel_x = 0.0;
float twist_vel_th = 0.0;
bool scan_go = true;
bool flag_go = false;

typedef union//发送结构体
{
    char  sp[8];
    float Vel[2];
}CANSPEED;

typedef union//接收结构体
{
    char  sp[8];
    float Vel[2];
}rxcan;



CANSPEED CanVel;
rxcan CanRec;


void *receive_func(void* param)  //接收线程。
{
    int reclen=0;
    int count=0;//数据列表中，用来存储列表序号。
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i,j;

    int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
    while((*run)&0x0f)
    {
        try
        {
            if((reclen=VCI_Receive(VCI_USBCAN1,can1,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
            {
                j=0;
                {
                    if(rec[j].ID==0x181 or rec[j].ID==0x182)
                    {
                        // can_time = ros::Time::now().toSec();
                        // printf("Index:%04d  ",count);count++;//序号递增
                        // printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                        // if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                        // if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                        // if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
                        // if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                        // printf("DLC:0x%02X",rec[j].DataLen);//帧长度
                        // printf(" data:0x");  //数据

                        // for(i = 0; i < rec[j].DataLen; i++)
                        // {
                        //     printf(" %02X", rec[j].Data[i]);
                        // }
                        // printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
                        // printf("\n");

                        //break;
                        

                    }

                }
            }
            if((reclen=VCI_Receive(VCI_USBCAN1,can2,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
            {
                j=0;
                {
                    if(rec[j].ID==0x10F8109A)
                    {
                        // can_time = ros::Time::now().toSec();
                        // printf("Index:%04d  ",count);count++;//序号递增
                        printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                        if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                        if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                        if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
                        if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                        printf("DLC:0x%02X",rec[j].DataLen);//帧长度
                        printf(" data:0x");  //数据

                        for(i = 0; i < rec[j].DataLen; i++)
                        {
                            printf(" %02X", rec[j].Data[i]);
                        }
                        printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
                        printf("\n");

                        //break;
                        

                    }

                }
            }
            // if ((ros::Time::now().toSec() - can_time) > 0.5) 
            // {
            //     speed_v = 0;
            //     speed_w = 0;
            //     cout<<"驱动器断电--------speed_v: " <<speed_v <<" speed_w: "<<speed_w<<endl;
            // }
        }
        catch(ros::TimeNotInitializedException& te)
        {
            printf("ros时间获取异常\n");
        }

        // sleep(0.02);
    }
    printf("run thread exit\n");//退出接收线程
    pthread_exit(0);
}


void cmd_velCallback(const geometry_msgs::Twist &twist_aux)//遥控速度控制回调
{
    vel_x = twist_aux.linear.x;  //设置速度大小
    vel_th = -twist_aux.angular.z;
    brake = twist_aux.angular.x;
    cmd_start = twist_aux.linear.z;  //遥控是否介入
    printf("cmd_vel----------vel_x: %f vel_th: %f \n",vel_x,vel_th);
}

void twist_cmdCallback(const geometry_msgs::TwistStamped &twist_aux)//自动驾驶速度控制回调
{
    twist_vel_x = twist_aux.twist.linear.x / 5.555;  //设置速度大小
    twist_vel_th = twist_aux.twist.angular.z/0.4 +0.07;
    printf("twist_cmd----------vel_x: %f vel_th: %f \n",vel_x,vel_th);
}

void can_open(const int &num)
{
	printf(">>this is hello !\r\n");//指示程序已运行
    if(VCI_OpenDevice(VCI_USBCAN1,num,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error!\n");
     //    VCI_CloseDevice(VCI_USBCAN1,num);//关闭设备。
    	// // VCI_CloseDevice(VCI_USBCAN1,1);//关闭设备。
     //    exit(1);
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN1,num,&pInfo)==1)//读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
     //    VCI_CloseDevice(VCI_USBCAN1,num);//关闭设备。
    	// // VCI_CloseDevice(VCI_USBCAN1,1);//关闭设备。
     //    exit(1);
    }
    cout<<"pInfo: "<< pInfo.str_Serial_Num<<"\n";
    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;//FFFFFFFF全部接收
    config.Filter=1;//接收所有帧  2-只接受标准帧  3-只接受扩展帧
    if (strncmp("019100007A5",pInfo.str_Serial_Num,20))
    {
    	config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
    	can1 = num;
    	// printf("111111111111111111111111111\n");
    }
    else if (strncmp("019100007A7",pInfo.str_Serial_Num,20))
    {
    	config.Timing0=0x00; /*波特率500 Kbps  0x00  0x1C*/
    	can2 = num;
    	// printf("22222222222222222222222222\n");
    }
    config.Timing0=0x00;
    config.Timing1=0x1C;
    config.Mode=0;//正常模式
    if(VCI_InitCAN(VCI_USBCAN1,num,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN1,0);
    }

    if(VCI_StartCAN(VCI_USBCAN1,num,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN1,0);
    }
    // printf(">>CAN1 opening\n");
}
void scanCallback(const sensor_msgs::LaserScan &scan_aux)
{

    //原来避障方案
    /*
    //雷达避障
    float temp=5.0;
    int index =0;
    sensor_msgs::LaserScan scan = scan_aux;

    cout << "scan_aux:"<< sizeof(scan_aux.ranges)<<endl;
    for(int i = 0;i < 225; i++)
    {
        if(temp > scan_aux.ranges[i])
        {
            temp = scan_aux.ranges[i];
            index = i;

        }

    }
    for(int i = 1775;i < 2000; i++)
    {
        if(temp > scan_aux.ranges[i])
        {
            temp = scan_aux.ranges[i];
            index = i;
        }
    }

    if(temp <= 4 && temp >=0.16)
    {
        cout << "i: "<< index << endl;
        cout << "distance: "<< temp << endl;
	scan_go = false;
        cout << "scan stop!!!"<< endl;
    }
    else
    {
	scan_go = true;
        cout << "-----------gogo!!!--"<< endl;
    }
    */


    //新的避障方案
	/*
	author: Taosit_Su  
	date: 2020-03-30
	因为雷达扫描的是园型，如果用来避障取固定返回距离，是个扇形避障区
	实际上使用是个矩形避障区域，以下代码完成，扇形到矩形避张区域的转换
	只需要修改参数：
	--degreeStart和degreeEnd是设定左边的扇形范围
	--degreeStartII和degreeEndII是设定右边的扇形范围
	--avoid_distances是需要的车身前方避障距离
	--car_width是车身宽度的一半，如果车宽2m，car_width设为1
	*/
    //参数配置
    const double avoid_distances = 2.0; //需要的车身前方避障距离
    const double car_width = 0.8;       //车身宽度的一半
    const double degreeStart = 0.0; //需要雷达前方的起始扇形范围
    const double degreeEnd = 90.0; //需要雷达前方的结束扇形范围

    const double degreeStartII = 270.0; //需要雷达前方的起始扇形范围
    const double degreeEndII = 360.0; //需要雷达前方的结束扇形范围
    //初始化
    const double PI = 3.1415926535;
    double preDegree4range = 0.0;
    double startI = 0.0;
    double endI= 0.0;
    double startII = 0.0;
    double endII= 0.0;
    double theta = atan(car_width / avoid_distances);//弧度制的theta
    double temp = 0.0;

    sensor_msgs::LaserScan scan = scan_aux;
    //发布一个避障区，显示用
    //在rviz订阅/scan_obstacle 的topic可以看到一个矩形的避障区域
    //如果矩形显示不完整，说明有障碍处于这个避障区域范围内，车会停
    sensor_msgs::LaserScan scan_pub = scan;
    for(int scan_num = 0;scan_num<scan.ranges.size();scan_num++)  
    {
    	scan_pub.ranges[scan_num] = 0.0;
    }
    preDegree4range = scan.ranges.size()/360.0; //1度需要几个range,不能写360,会取整
    startI = int(degreeStart * preDegree4range);//开始角度的数组下标
    endI  = int(degreeEnd * preDegree4range);//结束角度的数组下标
    startII = int(degreeStartII * preDegree4range);//开始角度的数组下标
    endII  = int(degreeEndII * preDegree4range);//结束角度的数组下标

    //左边的
    for(int i = startI;i < endI; i++)
    {
	if(i<int((theta*180/PI)*preDegree4range))
	{
		//生成一条横线的障碍,(i/degree)是当前角度，横线只覆盖车身宽度前方
       		temp = avoid_distances/cos((i/preDegree4range)*PI/180); 
		scan_pub.ranges[i] = temp;
		if(scan.ranges[i] < temp)
		{
			cout << "i: "<< i << endl;
			cout << "distance: "<< scan.ranges[i] << endl;
			scan_go = false;
			cout << "scan stop!!!"<< endl;
			break;
		}
	}else{
		//生成一条竖线的障碍,(i/degree)是当前角度，横线只覆盖avoid_distances的距离
       		temp = car_width/cos((90-(i/preDegree4range))*PI/180);
		scan_pub.ranges[i] = temp;
		if(scan.ranges[i] < temp)
		{
			cout << "i: "<< i << endl;
			cout << "distance: "<< scan.ranges[i] << endl;
			scan_go = false;
			cout << "scan stop!!!"<< endl;
			break;
		}
	}
	scan_go = true;

    }

    //右边的，这是假定单线雷达放在中间，如果雷达放在车右顶角就把右边的注释掉
    if(scan_go == true)
    {
	    for(int i = startII;i < endII; i++)
	    {
		if(i<int((360-theta*180/PI)*preDegree4range))
		{
			//生成一条竖线的障碍,(i/degree)是当前角度，横线只覆盖avoid_distances的距离
	       		temp = car_width/cos((i-startII)/preDegree4range*PI/180);
			scan_pub.ranges[i] = temp;
			if(scan.ranges[i] < temp)
			{
				cout << "i: "<< i << endl;
				cout << "distance: "<< scan.ranges[i] << endl;
				scan_go = false;
				cout << "scan stop!!!"<< endl;
				break;
			}
		}else{
			//生成一条横线的障碍,(i/degree)是当前角度，横线只覆盖车身宽度前方
	       		temp = avoid_distances/cos(((scan.ranges.size()-i)/preDegree4range)*PI/180);
			scan_pub.ranges[i] = temp;
			if(scan.ranges[i] < temp)
			{
				cout << "i: "<< i << endl;
				cout << "distance: "<< scan.ranges[i] << endl;
				scan_go = false;
				cout << "scan stop!!!"<< endl;
				break;
			}
		}
		scan_go = true;	
	    }
    }

    //发布主题scan_obstacle
    pub.publish(scan_pub);
    
}


/*
void scanCallback(const sensor_msgs::LaserScan &scan_aux)
{
    //参数配置
    const double avoid_distances = 2.0; //需要的车身前方避障距离
    const double car_width = 2.0;       //车身宽度
    const double degreeStart = 0.0; //需要雷达前方的起始扇形范围
    const double degreeEnd = 90.0; //需要雷达前方的结束扇形范围

    //初始化
    const double PI = 3.1415926535;
    double preDegree4range = 0.0;
    double startI = 0.0;
    double endI= 0.0;
    double theta = atan(car_width / avoid_distances);
    sensor_msgs::LaserScan scan = scan_aux;
    preDegree4range = scan.ranges.size()/360.0; //1度需要几个range,不能写360,会取整
    startI = int(degreeStart * preDegree4range);//开始角度的数组下标
    endI  = int(degreeEnd * preDegree4range);//结束角度的数组下标

    //开始计算
    for(int i = startI;i < endI; i++)
    {
	if(i<int((theta*180/PI)*preDegree4range))
	{
		//生成一条横线的障碍,(i/degree)是当前角度，横线只覆盖车身宽度前方
       		scan.ranges[i] = fabs(avoid_distances/cos((i/preDegree4range)*PI/180)); 
	}else{
		//生成一条竖线的障碍,(i/degree)是当前角度，横线只覆盖avoid_distances的距离
       		scan.ranges[i] = car_width/cos((90-(i/preDegree4range))*PI/180);
	}
   	std::cout<<"雷达只扫描车身前方的角度:"<<theta*180/PI<<"度"<<std::endl;
    }
    //发布主题scan_obstacle
    pub.publish(scan);
}
*/

static void flag_goCallback(const std_msgs::Int32::ConstPtr& input)
{
	if(input->data == 1)
	{	
		flag_go = true;
	}else{
		flag_go = false;
	}
}

int main(int argc, char **argv)
{
    int i=0;
    
    can_open(0);
    can_open(1);

    //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000088;
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;
    //需要发送的帧，结构体设置
    VCI_CAN_OBJ send2[1];
    send2[0].ID=0x00000211;
    send2[0].SendType=0;
    send2[0].RemoteFlag=0;
    send2[0].ExternFlag=0;
    send2[0].DataLen=8;
    //需要读取的帧，结构体设置
    VCI_CAN_OBJ rev[1];
    rev[0].SendType=0;
    rev[0].RemoteFlag=0;
    rev[0].ExternFlag=0;
    rev[0].DataLen=8;

    //需要读取的帧，结构体设置
    VCI_CAN_OBJ rev2[1];
    rev2[0].SendType=0;
    rev2[0].RemoteFlag=0;
    rev2[0].ExternFlag=0;
    rev2[0].DataLen=8;

    int m_run0=1;
    pthread_t threadid;
    int ret;
    ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
    int times = 5;

    ros::init(argc, argv, "base_can");
    ros::NodeHandle n;

    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 50, cmd_velCallback);//速度回调
    ros::Subscriber twist_cmd_sub = n.subscribe("/twist_cmd", 50, twist_cmdCallback);//速度回调
    ros::Subscriber scan_sub = n.subscribe("/scan", 100, scanCallback);//scan
    ros::Subscriber flag_go_sub = n.subscribe("/flag_go", 50, flag_goCallback);//速度回调
    //创建ROS的发布节点
    pub = nh.advertise<sensor_msgs::LaserScan> ("/scan_obstacle", 100);
    int gear = 0;//档位



	
    //ros::Rate loop_rate(30);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {	
    	if (vel_x > 0)  gear = 1;
    	else if (vel_x < 0)  gear = 2;
    	else  gear = 0;
//	if(scan_go == false||flag_go == false)
//	{
		//vel_x = 0;
//		twist_vel_x = 0;
//                twist_vel_th = 0;
//                brake = -1;
//	}
        //要写入的数据
	//原来的
        /*
	if(cmd_start == 1)
        {
            CanVel.sp[0] = int(vel_x*6000+10000)&255;
            CanVel.sp[1] = int(vel_x*6000+10000)>>8;
        }
        else if(scan_go == false||flag_go == false)
        {	
            CanVel.sp[0] = int(10000)&255;
            CanVel.sp[1] = int(10000)>>8;
            brake = -0.8;
        }
	else
	{
	    CanVel.sp[0] = int(twist_vel_x*6000+10000)&255;
	    CanVel.sp[1] = int(twist_vel_x*6000+10000)>>8;
 
	}
	*/
	//苏道改的
	if(cmd_start == 1)
        {
            CanVel.sp[0] = int(vel_x*6000+10000)&255;
            CanVel.sp[1] = int(vel_x*6000+10000)>>8;
        }
        else 
	{
		if(scan_go == false||flag_go == false)
		{	
		    CanVel.sp[0] = int(10000)&255;
		    CanVel.sp[1] = int(10000)>>8;
		    brake = -0.8;
		}
		else
		{
		    CanVel.sp[0] = int(twist_vel_x*6000+10000)&255;
		    CanVel.sp[1] = int(twist_vel_x*6000+10000)>>8;
	 
		}
	}
        CanVel.sp[2] = 0x00;
        CanVel.sp[3] = 0x00;
        CanVel.sp[4] = 0x00;
        CanVel.sp[5] = 0x00;
        CanVel.sp[6] = 0x00;
        CanVel.sp[7] = 0x00;
        CanVel.sp[7] = 0x00;
        //cout << hex<<vel<<" "<<(vel>>8&255)<<" "<<(vel&255)<<" "<<endl;
        // cout<< "CanVel.sp[7]: "<<int(CanVel.sp[1]) << " "<< vel_th*126+127<<endl;
        send[0].Data[0]=CanVel.sp[0];
        send[0].Data[1]=CanVel.sp[1];
        send[0].Data[2]=CanVel.sp[2];
        send[0].Data[3]=CanVel.sp[3];
        send[0].Data[4]=CanVel.sp[4];
        send[0].Data[5]=CanVel.sp[5];
        send[0].Data[6]=CanVel.sp[6];
        send[0].Data[7]=CanVel.sp[7];


        // for(int i = 0; i<8;i++)
        // {
        // 	cout<< "CanVel.sp["<<i<<"]: "<<int(CanVel.sp[i])<<endl;
        // }
        // printf("------------------------------------\n");

        //写入数据
        if(VCI_Transmit(VCI_USBCAN1, can1, 0, send, 1) == 1)
        {
           // printf("send----------------TX data successful!\n");
        }

        CanVel.sp[0] = 0x00;
	//原来的
	/*
	if(cmd_start == 1)
        {
            CanVel.sp[1] = vel_th*99+100;
        }
        else if(scan_go == false||flag_go == false)
        {
            CanVel.sp[1] = 100;
	    brake = -0.8;
        }
	else
	{
	    CanVel.sp[1] = twist_vel_th*99+100;
	}
	*/
	//苏道改的
	if(cmd_start == 1)
        {
            CanVel.sp[1] = vel_th*99+100;
        }
        else 
	{
		if(scan_go == false||flag_go == false)
		{
		    CanVel.sp[1] = 100;
		    brake = -0.8;
		}
		else
		{
		    CanVel.sp[1] = twist_vel_th*99+100;
		}
	}

        CanVel.sp[2] = 0x00;
        CanVel.sp[3] = 0x00;
        CanVel.sp[4] = 0x00;
        CanVel.sp[5] = 0x00;
        CanVel.sp[6] = 0x00;
        CanVel.sp[7] = 0x00;
        CanVel.sp[7] = int(fabs(brake-1.0)*127);;
        //cout << hex<<vel<<" "<<(vel>>8&255)<<" "<<(vel&255)<<" "<<endl;
        // cout<< "CanVel.sp[7]: "<<int(CanVel.sp[1]) << " "<< vel_th*126+127<<endl;
        send2[0].Data[0]=CanVel.sp[0];
        send2[0].Data[1]=CanVel.sp[1];
        send2[0].Data[2]=CanVel.sp[2];
        send2[0].Data[3]=CanVel.sp[3];
        send2[0].Data[4]=CanVel.sp[4];
        send2[0].Data[5]=CanVel.sp[5];
        send2[0].Data[6]=CanVel.sp[6];
        send2[0].Data[7]=CanVel.sp[7];
        //写入数据
        if(VCI_Transmit(VCI_USBCAN1, can2, 0, send2, 1) == 1)
        {
           // printf("send----------------TX data successful!\n");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    int num = 10;
    num = VCI_CloseDevice(VCI_USBCAN1,0);//关闭设备。
    printf("num1:%d\n",num );
    num = VCI_CloseDevice(VCI_USBCAN1,1);//关闭设备。
    printf("num2:%d\n",num );
    return 0;

    //除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
    //goto ext;
}

