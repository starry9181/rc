/**
  |-------------------------------- Copyright -----------------------------------|
  |                                                                              |
  |                        (C) Copyright 2019,����ƽͷ��,                         |
  |            1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
  |                            All Rights Reserved                               |
  |                                                                              |
  |            By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)          |
  |                     https://github.com/GCUWildwolfteam                       |
  |------------------------------------------------------------------------------|
  |--FileName    : gyro.h                                                
  |--Version     : v1.0                                                            
  |--Author      : ����ƽͷ��                                                       
  |--Date        : 2019-03-25               
  |--Libsupports : STM32CubeF1 Firmware Package V1.6.0(�ñ�Ŀ�����������)
  |--Description :                                                       
  |--FunctionList                                                       
  |-------1. ....                                                       
  |          <version>:                                                       
  |     <modify staff>:                                                       
  |             <data>:                                                       
  |      <description>:                                                        
  |-------2. ...                                                       
  |---------------------------------declaration of end----------------------------|
 **/
#ifndef __GYRO_H 
#define __GYRO_H 
#include "bsp_usart.h"
#include "bsp_can.h" 
typedef struct gy955Struct
{
	int16_t  		Gyr_X;
	int16_t  		Gyr_y;
	int16_t  		Gyr_z;
	float     Yaw;
	float			Roll;
	float 			Pitch;
  float      Gyrz;
	uint16_t       counter;
}gy955Struct;
typedef struct gyroStruct
{
  int16_t  		Gyrx;
  int16_t  		Gyrz;
  int16_t     Yaw;
  int16_t 	  Pitch;
  fps_t fps;
}gyroStruct;
void Gy955Init(UART_HandleTypeDef *huartx);
void AnalysisGyro(gy955Struct* Gyc);
void BingeGyroByCan(gyroStruct* Gyc,uint8_t *data);
#define GYRO_MAX_ANGLE  360//���������Ƕ�Ϊ360��
#endif	// __GYRO_H
  
 /*------------------------------------file of end-------------------------------*/


