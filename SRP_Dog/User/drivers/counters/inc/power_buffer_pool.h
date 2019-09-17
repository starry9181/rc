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
	|--FileName    : power_buffer_pool.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-03-24               
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
#ifndef __POWER_BUFFER_POOL_H 
#define __POWER_BUFFER_POOL_H 
#include "baseclass.h "
#include "currentmeter.h"
#include "communicate.h"
typedef struct powerBufferPoolStruct
{
	currentMeterStruct* pcurrentMeter_t;
    refereeSystemStruct* p_refereeSystem_t;
	float max_p;
	float max_w;//���ʵ�λW
	float r_w;//���ʵ�λW
  float r_p;//���ʵ�λW
	float current_mapp_coe;//����ӳ��ϵ��
	float high_water_level;
	float low_water_level;
	float mid_water_level;
	float period;//�������ڣ���λ/s
	float high_current_threshold;//A
  float mid_current_threshold;//A
  float low_current_threshold;//A
  float safe_current_threshold;//A
}powerBufferPoolStruct;
float OutMapCurrent(float coe,int16_t input);
	int16_t CurrentMapOut(float coe,float current);
    uint8_t Inject(powerBufferPoolStruct* pbs);
int16_t GetOutlet(powerBufferPoolStruct* pbs,int16_t input);
  uint8_t GetPowerPoolState(powerBufferPoolStruct* pbs);
void SetInPut(powerBufferPoolStruct* pbs,int16_t *input,uint8_t len);
#endif	// __POWER_BUFFER_POOL_H
/*-----------------------------------file of end------------------------------*/


