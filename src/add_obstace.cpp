/*
author: Taosit_Su  
date: 2020-03-30
因为雷达扫描的是原型，如果用来避障取固定返回距离，是个扇形避障区
实际上使用是个矩形避障区域，以下代码完成，扇形到矩形避张区域的转换
只需要修改参数：
--degreeStart和degreeEnd是设定区的扇形范围
--avoid_distances是需要的车身前方避障距离
--car_width是车身宽度
*/

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>






//申明发布器
ros::Publisher pub;

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
    //发布主题scan1
    pub.publish(scan);
}


int main (int argc, char** argv)
{
  // 初始化 ROS节点
  ros::init (argc, argv, "add_obstace");
  ros::NodeHandle nh;   //声明节点的名称

  // 为接受点云数据创建一个订阅节点
  ros::Subscriber scan_sub = nh.subscribe("/scan", 100, scanCallback);
  //创建ROS的发布节点
  pub = nh.advertise<sensor_msgs::LaserScan> ("/scan1", 100);

  // 回调
  ros::spin ();
}
