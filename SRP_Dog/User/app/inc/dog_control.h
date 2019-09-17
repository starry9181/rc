/**
	|--------------------------------- Copyright --------------------------------|
	|                                                                            |
	|                      (C) Copyright 2019,海康平头哥,                         |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                           All Rights Reserved                              |
	|                                                                            |
	|           By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : dog_control.h                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2019-09-10               
	|--Libsupports : 
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __DOG_CONTROL_H 
#define __DOG_CONTROL_H
#include "counters.h" 
#include "DJI_dbus.h"  
/*------------------------------------  宏定义  -------------------------------*/
#define D_RUNING   0 //行走
#define D_STARING  1//初始状态
#define D_GAIT_OK  2//一个步态完成
#define MAX_LEG_HIGH 30//最大抬腿高度 单位mm
#define MIN_LEG_HIGH 0//最小抬腿高度  单位mm
#define LEG_F_CTRL_PIONT_S  (1.0f/3.0f)//抬腿曲线控制点占比 在1/3处最合适
#define BEZIER_BUFF_DATE_T 100//贝塞尔曲线精度
#define B_L_LEG      0//前左腿
#define B_R_LEG      1//前右腿
#define A_L_LEG      2//后左腿
#define A_R_LEG      3//后右腿
#define B_L_AND_A_R  1//前左和后右
#define B_R_AND_A_L  0//前右跟后左
/*---------------------------------  结构体声明  ------------------------------*/
typedef struct dogCtrlStruct
{
	int16_t high;//抬腿高度
	int16_t speed;//抬腿速度
  PointU32 in_t[3];//控制点
  PointU32 out_t[BEZIER_BUFF_DATE_T]; //贝塞尔规划缓存空间
} dogCtrlStruct;
typedef struct geometricForIK
{
  /*基准方向角：逆时针为正*/
  float q1;   //l1初始角度  单位 度
  float q2;   //l2初始角度  单位 度
  float l1;   //大腿长度  单位mm
  float l2;   //小腿长度  单位mm
  float psi;  //ψ
  float last_q1; //q1上次角度  单位 度 初始化时一定要  last_q1 = q1
  uint8_t status;//初始化状态 MOD_READ为未初始化，INIT_OK初始化；
  /*初始化示例
   gIK_t.q1 = 45;   //l1初始角度  45°
   gIK_t.q2 = 45;   //l2初始角度  45°
   gIK_t.l1 = 100;   //大腿长度 100mm
   gIK_t.l2 = 100;   //小腿长度 100mm
   gIK_t.psi = 0;  //ψ
   gIK_t.last_q1 = gIK_t.q1; //q1上次角度  单位 度 初始化时一定要  last_q1 = q1
   gIK_t.status = INIT_OK;//初始化状态 MOD_READ为未初始化，INIT_OK初始化
  */
}geometricForIK;
/*-----------------------------------  函数声明  ------------------------------*/
void DogControlInit(const dbusStruct* pRc_t);
void DogCtrl(void);
uint8_t TrajectoryPlanning(int16_t speed,int16_t high);
void CalBezierCtrlPoint(int16_t speed,int16_t high);
uint8_t FootControls(uint8_t w_leg);
SYS_Status GeometricForIK(geometricForIK *gIK_t,PointU32 point);
#endif
/*-----------------------------------file of end------------------------------*/


