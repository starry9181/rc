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
	|--FileName    : dog_control.c                                              
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
#include "dog_control.h"
#include <Math.h>
dogCtrlStruct dogCtrl_t;
geometricForIK gIK_t;
redifRcStruct dogCtrlRedifRc_t; //按键重定义
	/**
		* @Data    2019-09-10 19:30
		* @brief   初始化
		* @param   void
		* @retval  void
		*/
		void DogControlInit(const dbusStruct* pRc_t)
		{
			dogCtrl_t.high = 0;
      dogCtrl_t.speed = 0;
      dogCtrlRedifRc_t.pRc_t = pRc_t; 
      dogCtrlRedifRc_t.ch1 = NULL;
      dogCtrlRedifRc_t.ch2 = NULL;
      dogCtrlRedifRc_t.ch3 = &dogCtrl_t.speed;
      dogCtrlRedifRc_t.ch4 = &dogCtrl_t.high;
      dogCtrlRedifRc_t.switch_left = NULL;
      dogCtrlRedifRc_t.switch_right = NULL;
      dogCtrlRedifRc_t.thumbwheel = NULL;
      
      dogCtrl_t.in_t[0].X = 0;
      dogCtrl_t.in_t[0].Y = 0;
      dogCtrl_t.in_t[1].X = 0;
      dogCtrl_t.in_t[1].Y = 0;
      dogCtrl_t.in_t[2].X = 60;
      dogCtrl_t.in_t[2].Y = 0;
      gIK_t.q1 = 45;   //l1初始角度  45°
      gIK_t.q2 = 45;   //l2初始角度  45°
      gIK_t.l1 = 100;   //大腿长度 100mm
      gIK_t.l2 = 100;   //小腿长度 100mm
      gIK_t.psi = 0;  //ψ
      gIK_t.last_q1 = gIK_t.q1; //q1上次角度  单位 度 初始化时一定要  last_q1 = q1
      gIK_t.status = INIT_OK;
		}
/**
	* @Data    2019-09-10 19:38
	* @brief   运动控制
	* @param   void
	* @retval  void
	*/
#ifdef DEBUG_BY_KEIL
int16_t temp_speed=0,temp_high=0;
    uint32_t temp_buff,temp_q1,temp_q2;
#endif
	void DogCtrl(void)
	{
#ifdef DEBUG_BY_KEIL
    DefineRcKey(&dogCtrlRedifRc_t);
    TrajectoryPlanning(dogCtrl_t.speed,(int16_t)(dogCtrl_t.high * 0.045));
    for(int i=1;i<100;i++)
    {
       GeometricForIK(&gIK_t,dogCtrl_t.out_t[i]);
       temp_q1 = (uint32_t)gIK_t.q1*10;
       temp_q2 = (uint32_t)gIK_t.q2*10;
//      if(dogCtrl_t.out_t[i].X > dogCtrl_t.out_t[i-1].X) 
//      {
//        temp_buff = dogCtrl_t.out_t[i].Y;
//        osDelay(4);
//      }        
    }
           osDelay(100);
		//CalBezierCtrlPoint(dogCtrl_t.speed,(int16_t)(dogCtrl_t.high * 0.0454f));
#endif
	}
  uint8_t d_state=0;
/**
	* @Data    2019-09-10 19:40
	* @brief  对角步态
	* @param   void
	* @retval  void
	*/
  uint8_t diagonal_state = B_R_AND_A_L;//初始化时位前右脚先出
 void DiagonalGait(int16_t speed,int16_t high)
 {
    TrajectoryPlanning(speed,high);
   if(d_state == D_STARING)//开始起步
   {
     for(int i=0;i<BEZIER_BUFF_DATE_T;i++)
     {
       //GeometricForIK(&gIK_t,i,dogCtrl_t.buff_b_datex[i]);
       //if(FootControls(B_R_LEG) ==D_GAIT_OK && FootControls(A_L_LEG) ==D_GAIT_OK)
     }
        d_state = D_RUNING;//标位行走状态
   }
   else
   {
     
   }      
 }   
/**
	* @Data    2019-09-10 19:45
	* @brief   脚运动控制
	* @param   int16_t speed 抬腿速度
  * @param   int16_t high  抬腿高度
	* @retval  void
	*/
uint8_t FootControls(uint8_t w_leg)
{
  switch (w_leg)
  {
    case B_L_LEG:
       //GeometricForIK(dogCtrl_t)
      break;
    case B_R_LEG:
      break;
    case A_L_LEG:
      break;
    case A_R_LEG:
      break;
  }
  return D_GAIT_OK;
}
/**
	* @Data    2019-09-10 19:45
	* @brief   腿高度换算成贝塞尔控制点  公式: P1y = Py/2(1-t)t
  * @param   int16_t high  抬腿高度
	* @retval  void
	*/
void CalBezierCtrlPoint(int16_t speed,int16_t high)
{
  /*限制最大最小抬腿高度*/
  high = MAX(high,MAX_LEG_HIGH);
  high = MIN(high,MIN_LEG_HIGH);
  dogCtrl_t.in_t[1].Y  = (uint32_t)(high/(2*(1-LEG_F_CTRL_PIONT_S)*LEG_F_CTRL_PIONT_S));
}
/**
	* @Data    2019-09-11 12:37
	* @brief   脚运动控制
	* @param   int16_t speed 抬腿速度
  * @param   int16_t high  抬腿高度
	* @retval  void
	*/
uint8_t TrajectoryPlanning(int16_t speed,int16_t high)
{
  CalBezierCtrlPoint(speed,high);//高度换算成控制点
  CreateBezierCurves(dogCtrl_t.in_t,3,dogCtrl_t.out_t,BEZIER_BUFF_DATE_T);//计算规划轨迹
  return D_GAIT_OK;
}
/**
	* @Data    2019-09-12 14:52
	* @brief   两轴机器人逆运动学求解 几何法
	* @param   uint32_t x 目标点x
  * @param   uint32_t y 目标点y
	* @retval  SYS_Status	
	*/
	SYS_Status GeometricForIK(geometricForIK *gIK_t,PointU32 point)
	{
    float s = (float)(point.X *point.X  + point.Y*point.Y);            
    float temp1=0,temp2=0,q1_temp1=0,q1_temp2=0;
    if(!IS_INIT_OK(gIK_t->status)) //判断是否初始化成功
      return SYS_ERROR;
    /*求Q2角度*/
		gIK_t->q2 = PI - (((gIK_t->l1 * gIK_t->l1) + (gIK_t->l2 * gIK_t->l2) -  \
                        s)/(2 * gIK_t->l1 * gIK_t->l2));
    gIK_t->q2 = acos(gIK_t->q2);
    /*求ψ角度*/
    gIK_t->psi = acos((((gIK_t->l1 * gIK_t->l1) + s) - (gIK_t->l2 * gIK_t->l2)) \
                         / (2 * gIK_t->l1 * sqrt(s)));
    /*判断Q2的方向*/
    q1_temp1 = atan2(point.Y,point.X) - gIK_t->psi;
    q1_temp2 = atan2(point.Y,point.X) + gIK_t->psi;
    temp1 = q1_temp1 - gIK_t->last_q1;
    temp2 = q1_temp2 - gIK_t->last_q1;
    if(ABS(temp1) < ABS(temp2))
    {
      gIK_t->q1 = q1_temp1;  
    }
    else
    {
      gIK_t->q1 = q1_temp2;
      gIK_t->q2 = 360.f-gIK_t->q2;
    }
    /*更新上次缓存值*/    
    gIK_t->last_q1 = gIK_t->q1; 
    return SYS_OK;
	}
/*-----------------------------------file of end------------------------------*/


