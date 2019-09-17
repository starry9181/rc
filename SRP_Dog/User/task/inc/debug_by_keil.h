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
	|--FileName    : debug_by_keil.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-02-23               
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
#ifndef __DEBUG_BY_KEIL_H 
#define __DEBUG_BY_KEIL_H 
#include "baseclass.h " 
#ifdef DEBUG_BY_KEIL
#include "leds_tip.h" 
#include "parse.h" 

typedef struct debugByKeilStruct
{
// const dbusStruct* d_rc;
 // const chassisStruct* d_pchassis_t;
  int g;
}debugByKeilStruct;
void DebugByKeilInit(void);
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern debugByKeilStruct pdebug_t;
#endif 
#endif	// __DEBUG_BY_KEIL_H
/*-----------------------------------file of end------------------------------*/


