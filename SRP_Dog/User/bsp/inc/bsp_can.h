/**
	|-------------------------------- Copyright ---------------------------------|
	|                                                                            |
	|                       (C) Copyright 2019,����ƽͷ��,                        |
	|          1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                          All Rights Reserved                               |
	|                                                                            |
	|           By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)         |
	|                     https://github.com/GCUWildwolfteam                     |
	|----------------------------------------------------------------------------|
	|--FileName    : bsp_can.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-01-18               
	|--Libsupports : 
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-------------------------------declaration of end---------------------------|
 **/
#ifndef __BSP_CAN_H 
#define __BSP_CAN_H 
#include "baseclass.h " 
#include "fps.h" 
typedef struct canDataStrcut
{
	CAN_TxHeaderTypeDef txMsg; //����
	CAN_RxHeaderTypeDef rxMsg; //����
	CAN_FilterTypeDef filter;  //ɸѡ��
	uint8_t rxdata[8];         //���ջ�������
} canDataStrcut;
SYS_Status UserCanConfig(void);
SYS_Status CanFilterInit(CAN_HandleTypeDef *hcanx);
SYS_Status CanTxInit(CAN_HandleTypeDef *hcanx);
SYS_Status CanRxInit(CAN_HandleTypeDef *hcanx);
SYS_Status AllocateCanxSpace(CAN_HandleTypeDef *hcanx);
canDataStrcut* GetCanAddr(CAN_HandleTypeDef *hcanx);
SYS_Status CanQueueCreate(canDataStrcut *canx,uint8_t len,uint8_t deep);
SYS_Status UserCanQueueRX(CAN_HandleTypeDef *hcanx,\
																		void* const pvBuffer);
SYS_Status CanTxMsg(CAN_HandleTypeDef* hcanx,int id,uint8_t *message);		
void can1_rx(uint32_t id,uint8_t *data);
void can2_rx(uint32_t id,uint8_t *data);
#endif // __BSP_CAN_H

/*-----------------------------------file of end------------------------------*/


