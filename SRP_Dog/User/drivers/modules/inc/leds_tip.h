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
	|--FileName    : leds_tip.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-01-18               
	|--Libsupports : STM32CubeF4 Firmware Package V1.6.0(�ñ�Ŀ�����������)
	|--Description : �޸����ŵĺ�,��ͬ������Ų�һ��
	|--							 1��RM_NEW_BOARD
	|--							 2��RM_OLD_BOARD                                                      
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|---------------------------------declaration of end----------------------------|
 **/
#ifndef __LEDS_TIP_H 
#define __LEDS_TIP_H 
#include "baseclass.h "
#define TIP_BASE_TIME    10//ʱ��Ƭ10ms
	void FlashingLed(GPIO_TypeDef *GPIO, uint16_t ledx, uint8_t times, uint32_t lag);
	void ProgressBarLed(GPIO_TypeDef *GPIO, uint32_t lag);
   void WarningLed(uint8_t times,uint16_t lag);
     void SidesToMidLed(uint16_t speed);
     void MidToSidesLed(uint16_t speed); 
     void PutOutMidToSidesLed(uint16_t speed);
 void PutOutSidesToMidLed(uint16_t speed);
#endif // __LEDS_TIP_H

/*------------------------------------file of end-------------------------------*/


