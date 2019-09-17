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
  |--FileName    : pc_data.h                                                
  |--Version     : v1.0                                                            
  |--Author      : ����ƽͷ��                                                       
  |--Date        : 2019-03-21               
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
#ifndef __PC_DATA_H 
#define __PC_DATA_H 
#include "bsp_usart.h"
#include "DataStructure.h" 
#include "fps.h" 
typedef struct pcDataStruct
{
  uint32_t status;
  int16_t yaw_target_angle;
  int16_t pitch_target_angle;
  uint8_t commot;
  uint8_t shoot_commot;
  int16_t tem_fps;
  fps_t fps;
  int16_t distance;//��λcm
}pcDataStruct;
     void Pc_ParseData(pcDataStruct* pc);
     HAL_StatusTypeDef PcDataRxInit(pcDataStruct* pc);

int16_t YawDataConversion(int16_t yaw);
int16_t PitchDataConversion(int16_t pitch);
void EscPc(int16_t key,int16_t ch1,int16_t ch2,int16_t ch3,int16_t ch4,int16_t thumbwheel,int16_t key1);
void GetGyroAngle(int16_t yaw_angle,int16_t pitch_angle);
#define INTEL_YAW    490
#define INTEL_PITCH   305
#define GEN_YAW      270
#define GEN_PITCH     290
#define IND_YAW      650
#define IND_PITCH    600
#endif	// __PC_DATA_H
  
 /*------------------------------------file of end-------------------------------*/


