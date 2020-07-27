/*
author: Taosit_Su  
date: 2020-03-30
因为雷达扫描的是原型，如果用来避障取固定返回距离，是个扇形避障区
实际上使用是个矩形避障区域，以下代码完成，扇形到矩形避张区域的转换
只需要修改参数：
--degreeStart和degreeEnd是设定左边的扇形范围
--degreeStartII和degreeEndII是设定右边的扇形范围
--avoid_distances是需要的车身前方避障距离
--car_width是车身宽度的一半，如果车宽2m，car_width设为
*/

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>


bool scan_go = true;

using namespace std;

//申明发布器
ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScan &scan_aux)
{
	/*
    //参数配置
    const double avoid_distances = 2.0; //需要的车身前方避障距离
    const double car_width = 1.0;       //车身宽度
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
       		scan.ranges[i] = fabs(avoid_distances/cos((i/preDegree4range)*PI/180)); 
	}else{
		//生成一条竖线的障碍,(i/degree)是当前角度，横线只覆盖avoid_distances的距离
       		scan.ranges[i] = car_width/cos((90-(i/preDegree4range))*PI/180);
	}
   	std::cout<<"雷达只扫描车身前方的角度:"<<theta*180/PI<<"度"<<std::endl;
    }

    //右边的，这是假定单线雷达放在中间，如果雷达放在车右顶角就把右边的注释掉
    for(int i = startII;i < endII; i++)
    {
	if(i<int((360-theta*180/PI)*preDegree4range))
	{
		//生成一条竖线的障碍,(i/degree)是当前角度，横线只覆盖avoid_distances的距离
       		scan.ranges[i] = car_width/cos((i-startII)/preDegree4range*PI/180);
	}else{
		//生成一条横线的障碍,(i/degree)是当前角度，横线只覆盖车身宽度前方
       		scan.ranges[i] = fabs(avoid_distances/cos(((scan.ranges.size()-i)/preDegree4range)*PI/180));
	}
   	//ROS_INFO("--------%d",scan.ranges.size());
	
    }

    //发布主题scan1
    pub.publish(scan);
	*/

    //参数配置
    const double avoid_distances = 2.0; //需要的车身前方避障距离
    const double car_width = 0.8;       //车身宽度
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

    ROS_INFO("Final scan_go value : %d ",scan_go);

    //发布主题scan_obstacle
    pub.publish(scan_pub);
}


int main (int argc, char** argv)
{
  // 初始化 ROS节点
  ros::init (argc, argv, "avoid_obstace");
  ros::NodeHandle nh;   //声明节点的名称

  // 为接受点云数据创建一个订阅节点
  ros::Subscriber scan_sub = nh.subscribe("/scan1", 100, scanCallback);
  //创建ROS的发布节点
  pub = nh.advertise<sensor_msgs::LaserScan> ("/scan_obstacle", 100);

  // 回调
  ros::spin ();
}
