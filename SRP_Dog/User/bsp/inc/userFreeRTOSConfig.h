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
	|--FileName    : userFreeRTOSConfig.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-03-16               
	|--Libsupports : 
	|--Description : ����FreeRTOS�ӿ�
  |               �û�FreeRTOS���ã���ͷ�ļ�����FreeRTOSConfig.h��������û������                                                      
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __USERFREERTOSCONFIG_H 
#define __USERFREERTOSCONFIG_H 
//#include "SEGGER_SYSVIEW_FreeRTOS.h"
//#define INCLUDE_xTaskGetIdleTaskHandle  1
//#define INCLUDE_pxTaskGetStackStart     1
/* -------------- �����ʱ�� ----------------- */
//�Ƿ���붨ʱ����ش��룬 ����Ҫʹ�ö�ʱ���� ����Ϊ 1 
#define configUSE_TIMERS 1
//���ö�ʱ��Daemon �������ȼ��� ������ȼ�̫�ͣ� ���ܵ��¶�ʱ���޷���ʱִ�� 
#define configTIMER_TASK_PRIORITY  (2)
//���ö�ʱ��Daemon ��������������ȣ� ���ö�ʱ������ͨ��������Ϣ���ö���ʵ�ֵġ� 
#define configTIMER_QUEUE_LENGTH   10
//���ö�ʱ��Daemon �����ջ��С
#define configTIMER_TASK_STACK_DEPTH  256
	/* ----------------- �����ջ��С���� -------------------- */
   #define SYS_INIT_HEAP_SIZE          512//ϵͳ��ʼ����
   #define PARSE_HEAP_SIZE             1024//���ݽ�������
   #define DOG_CONTROL_HEAP_SIZE       1024//����������
  #define DEBUG_HEAP_SIZE              1024//Ӳ������������ʾ����
	 #define TX_HEAP_SIZE                512//�û���������
   #define QUEUE_HEAP_SIZE             512//����
	/* -------------- ���ж�ջ��С���� ----------------- */   
	/* ----------------- �����ڴ��С����-------------------- */
   #define TOTAL_HEAP_SIZE 								\
	         ((size_t)((										\
						 SYS_INIT_HEAP_SIZE + 				\
					   PARSE_HEAP_SIZE +						\
						 DOG_CONTROL_HEAP_SIZE +		\
						 TX_HEAP_SIZE +               \
             QUEUE_HEAP_SIZE +             \
						 configTIMER_TASK_STACK_DEPTH   \
																			 ) * 7))
                                       
                                       //	 DEBUG_HEAP_SIZE +
		

/* -------------- �ض���freertos�����ڴ��С ----------------- */
  #ifdef configTOTAL_HEAP_SIZE
    #undef configTOTAL_HEAP_SIZE
  #endif
  #ifndef configTOTAL_HEAP_SIZE
    #define configTOTAL_HEAP_SIZE   TOTAL_HEAP_SIZE//���¶����������ڴ��С����ֹcubemx����֮���޸�
  #endif
#endif	// __USERFREERTOSCONFIG_H
/*-----------------------------------file of end------------------------------*/


