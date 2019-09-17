/**
	|----------------------------- Copyright ------------------------------------|
	|                                                                            |
	|                        (C) Copyright 2018,����ƽͷ��,                       |
	|         1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China        |
	|                         All Rights Reserved                                |
	|                                                                            |
	|        By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)            |
	|                  https://github.com/GCUWildwolfteam                        |
	|----------------------------------------------------------------------------|
	|--FileName    : "input_sw.h                                              
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2018-11-27               
	|--Description :
  |                stm32F4xx_it.c���û�������                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|----------------------------declaration of end------------------------------|
**/
#ifndef __INPUT_SW_H 
#define __INPUT_SW_H
#include "baseclass.h" 
#define LIM_SW_TIME_OF_DURATION    400//20s =5ms*400
#define LASER_SW_TIME_OF_DURATION    50//0.5s =5ms*50
	uint8_t LaserSw(void);
   	uint8_t LaserSwBack(void);
     	uint8_t LaserSwBef(void);
uint8_t LimSw(void);
#endif
