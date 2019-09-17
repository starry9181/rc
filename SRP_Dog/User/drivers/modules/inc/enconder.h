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
	|--FileName    : enconder.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-03-14               
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
#ifndef __ENCONDER_H 
#define __ENCONDER_H 
#include "baseclass.h "
typedef struct incrementalEnconderStruct
{
	uint32_t last_data;//��¼�ϴε�CNTֵ
	int16_t counter;
	float coefficient;//ÿ���׵��������ĵ���
}incrementalEnconderStruct;
	HAL_StatusTypeDef EnconderInit(incrementalEnconderStruct* ies,uint16_t radius, int16_t poles);
	int32_t GetPosition(incrementalEnconderStruct* ies);
  void SetEncoderZeroPoint(incrementalEnconderStruct* ies,uint32_t zero);
MOD_Status CalibratingEncoder(incrementalEnconderStruct* ies,int32_t distanct);
  #define SET_ENCODER_ZERO(__zero)  (ENCOER_TIM->Instance->CNT = (__zero))
#endif	// __ENCONDER_H
/*-----------------------------------file of end------------------------------*/


