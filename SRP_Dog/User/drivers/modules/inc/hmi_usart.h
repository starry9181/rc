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
	|--FileName    : hmi_usart.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-08-07               
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
#ifndef __HMI_USART_H 
#define __HMI_USART_H 
#include "bsp_usart.h"
#include "fps.h" 
/* -------------- �궨�� ----------------- */
  #define PAGE_SIZE  11      //������ҳ����
  #define BUFFER_DATA_HMI (10+HEAD_FRAME_LEN) //��������
  #define LOCK_P	0xFF   //��������
  #define MAIN_P  0      //������
  #define P1_P    1     //����Ҫ��1����
  #define P2_P    2     //����Ҫ��2����
  #define P3_P    3     //����Ҫ��3����
  #define F1_P    4     //���Ӳ���1����
  #define F2_P    5     //���Ӳ���2����
  #define F3_P    6     //���Ӳ���3����
  #define PAGINATION_ID ('p')//ҳ���ʶ��
  #define STARTUP_ID    ('s')//���������ʶ��
  #define DISTANCE_ID   ('d')//�����ʶ��
  #define ANGEL_ID      ('a')//�Ƕȱ�ʶ��
  #define VAL_END_ID    (';')//����������ʶ�� 
  #define POINT_ID      ('.')//С�����ʶ�� 
  #define FUHAO         ('-')
  #define START_FLAG 1  //������־λ
/* -------------- �ṹ�� ----------------- */
	typedef struct hmiStrct
	{
    uint8_t status;
		uint8_t page;
    uint8_t data[BUFFER_DATA_HMI];
    uint8_t commot;
      uint8_t load;
    int16_t distance;
    float angle;
    fps_t fps;
	}hmiStrct;
  
	HAL_StatusTypeDef HmiUsartInit(hmiStrct* hmi);
  void HmiUsartParseData(hmiStrct* hmi);
#endif	// __HMI_USART_H
/*-----------------------------------file of end------------------------------*/


