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
	|--FileName    : currentmeter.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-03-23               
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
#ifndef __CURRENTMETER_H 
#define __CURRENTMETER_H 
#include "bsp_can.h" 
typedef struct
{
	uint8_t data_buff;
	float volt;
	float current;
	float power;
	float power_buffer;
  fps_t fps;
}currentMeterStruct;
uint8_t CurrentMeterAnalysis(currentMeterStruct *Cms,uint8_t *data);
#endif	// __CURRENTMETER_H
/*-----------------------------------file of end------------------------------*/


