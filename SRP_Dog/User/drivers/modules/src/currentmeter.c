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
	|--FileName    : currentmeter.c                                              
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
#include "currentmeter.h" 
	  /**
		* @Data    2018-12-27 15:35
		* @brief   ������can���Ľ���
		* @param   void
		* @retval  uint8_t
		*/
		uint8_t CurrentMeterAnalysis(currentMeterStruct *Cms,uint8_t *data)
		{
			Cms->volt = (float)((data[1]<<8 | data[0])/100.0);
			Cms->current = (float)((data[3]<<8 | data[2])/100.0);
			Cms->data_buff = (float)((data[5]<<8 | data[4])/100.0);
			Cms->power_buffer = (float)((data[7] | data[6])/100.0);
			return 0;
		}
/*-----------------------------------file of end------------------------------*/


