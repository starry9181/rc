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
	|--FileName    : fps.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-04-23               
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
#ifndef __FPS_H 
#define __FPS_H 
#include "baseclass.h" 
/*------------------------------------  �궨��  -------------------------------*/
 #define FPS  0//֡��
 #define FPS_ADDR  1//֡�ʵ�ַ
/*---------------------------------  �ṹ������  ------------------------------*/
 typedef uint32_t fps_t[2];
/*-----------------------------------  ��������  ------------------------------*/
SYS_Status SetFpsAddress(uint32_t fps[]);
SYS_Status FpsUserTimeInit(void);
void Fps(uint32_t fps[]);
/*��ȡ֡��*/
static inline uint32_t GetFps(uint32_t fps[])
{
  return fps[FPS];
}
#endif	// __FPS_H
/*-----------------------------------file of end------------------------------*/


