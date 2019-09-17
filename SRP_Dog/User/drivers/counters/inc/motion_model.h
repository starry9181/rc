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
	|--FileName    : motion_model.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-02-17               
	|--Libsupports : 
	|--Description :  �����˶�ģ���㷨                                                     
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __MOTION_MODEL_H 
#define __MOTION_MODEL_H 
#include "baseclass.h " 
	void GetThreeMotionModeData(uint8_t radius,float theta);
	void ThreeWheelMotionModel(int16_t *motorspeed,const int16_t vx,const \
                                                  int16_t vy,const int16_t w);
#endif	// __MOTION_MODEL_H
/*-----------------------------------file of end------------------------------*/


