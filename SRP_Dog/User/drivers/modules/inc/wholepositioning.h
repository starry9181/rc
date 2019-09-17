/**
	|-------------------------------- Copyright ----------------------------------------|
	|                                                                                   |
	|                        (C) Copyright 2018,����ƽͷ��,                              |
	|            1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China            |
	|                            All Rights Reserved                                    |
	|                                                                                   |
	|            By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)               |
	|                     https://github.com/GCUWildwolfteam                            |
	|-----------------------------------------------------------------------------------|
	|--FileName    : wholepositioning.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2018-12-29               
	|-- Libsupports: STM32CubeF1 Firmware Package V1.6.0 / 17-May-2017(�ñ�Ŀ�����������)           
	|--Description : 1������WholePositioningInit�����ڴ�ͳ�ʼ��
	|								 2������WholePositioningStart�������ղ���������                                                     
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|---------------------------------declaration of end---------------------------------|
 **/
#ifndef __WHOLEPOSITIONING_H 
#define __WHOLEPOSITIONING_H 
		#include "bsp_usart.h"

		typedef struct wholePositionStruct
		{
			float z_angle;							//z��Ƕ�															
			float x_angle;							//x��Ƕ�
			float y_angle;							//y��Ƕ�
			float x_coordinate; 				//x��Ƕ�
			float y_coordinate; 				//y��Ƕ�
			float angular_acceleration;	//�Ǽ��ٶ�

			float	C_X_Coordinate;
			float	C_Y_Coordinate;
			float C_Z_Angle;

			uint8_t *p_beffur;    //��������ָ��
		  UART_HandleTypeDef* p_get_usartx;//�õ�����״ָ̬��
			 uint8_t status;  		//����״̬ //0�������ɹ� ���������ʧ��
			 int16_t rx_flag;     //���ձ��λ  1����������-x�����֡ (x=1~10)
		/* -------- ����ȫ����λ���������ֳ��ṹ��ָ�� --------- */
			 struct wholePositionStruct* p_lastdata_t; 
		}wholePositionStruct;
		#define WHOLE_POSION_LENGTH 28   				//ȫ����λһ֡���� 28���ֽ�
		#define WHOLE_POSION_BACK_LENFTH 1 			//�����
		HAL_StatusTypeDef  WholePositioningSetConfig(UART_HandleTypeDef* huart);
	  HAL_StatusTypeDef WholePositioningInit(wholePositionStruct* wps,UART_HandleTypeDef* huart);
		HAL_StatusTypeDef ParseWholePositioningInfo(wholePositionStruct* wps);
    HAL_StatusTypeDef WholePositioningStart(wholePositionStruct* wps,UART_HandleTypeDef* huart);
#endif	// __WHOLEPOSITIONING_H
	
 /*------------------------------------file of end------------------------------------*/


