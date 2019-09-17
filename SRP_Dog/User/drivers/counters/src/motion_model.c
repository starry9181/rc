/**
	|--------------------------------- Copyright --------------------------------|
	|                                                                            |
	|                      (C) Copyright 2019,����ƽͷ��,                         |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                           All Rights Reserved                              |
	|                                                                            |
	|           By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : motion_model.c                                              
	|--Version     : v1.0                                                          
	|--Author      : ����ƽͷ��                                                     
	|--Date        : 2019-02-17             
	|--Libsupports : 
	|--Description :�����˶�ģ���㷨                                                     
	|--FunctionList                                                     
	|-------1. ....                                                     
	|          <version>:                                                     
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#include "motion_model.h" 
#include "Math.h"
/* ======================== ȫ���������˶�ģ�� of begin ======================= */
#define COS_THETA 				 0
#define SIN_THETA 			   1
#define COS_60_MIN_THETA 	 2
#define SIN_60_MIN_THETA 	 3
#define COS_60_ADD_THETA 	 4
#define SIN_60_ADD_THETA	 5
#define RADIUS             6
float value[7];
	/**
	* @Data    2019-02-17 21:24
	* @brief  ��ȡ���Ӱ뾶���ȣ���������ͻ���������Ƕ����,�����˶�ģ����ز���
	* @param   uint8_t radius  ����ΪԲ�İ뾶
	* @param   char theta ��������ͻ���������Ƕ���� 
	* @retval  void
	*/
	void GetThreeMotionModeData(uint8_t radius,float theta)
	{
    //cos�����㲻׼ȷ
//		value[COS_THETA] = cos(theta);
//	 	value[SIN_THETA] = sin(theta);
//	 	value[COS_60_MIN_THETA] = -cos((60 - theta));
//	 	value[SIN_60_MIN_THETA] = sin((60 - theta));
//	 	value[COS_60_ADD_THETA] = -cos((60 + theta));
//	 	value[SIN_60_ADD_THETA] = -sin((60 + theta));
//	 	value[RADIUS] = radius;
    value[COS_THETA] = 1;
	 	value[SIN_THETA] = 0;
	 	value[COS_60_MIN_THETA] = -0.5;
	 	value[SIN_60_MIN_THETA] = 1.732050;
	 	value[COS_60_ADD_THETA] = -0.5;
	 	value[SIN_60_ADD_THETA] = -1.732050;
	 	value[RADIUS] = radius;
	}
	/**
	* @Data    2019-02-17 20:17
	* @brief  ȫ�������ֵ����˶�ģ��
	* @param   void
	* @retval  void
	*/
	void ThreeWheelMotionModel(int16_t *motorspeed,const int16_t vx,const \
																								int16_t vy,const int16_t w)
	{
  	*motorspeed = (int16_t)(value[COS_THETA]*vx + value[SIN_THETA]*vy + \
																									value[RADIUS]*w);
		*(motorspeed+1) = (int16_t)(value[COS_60_MIN_THETA]*vx + \
																value[SIN_60_MIN_THETA]*vy + value[RADIUS]*w);
		*(motorspeed+2) = (int16_t)(value[COS_60_ADD_THETA]*vx + \
																value[SIN_60_ADD_THETA]*vy + value[RADIUS]*w);
	}
/* ======================== ȫ���������˶�ģ�� of end ======================== */
/*-----------------------------------file of end------------------------------*/


