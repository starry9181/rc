/**
	|--------------------------------- Copyright --------------------------------|
	|                                                                            |
	|                      (C) Copyright 2018,����ƽͷ��,                         |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                           All Rights Reserved                              |
	|                                                                            |
	|           By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : user.c                                              
	|--Version     : v1.0                                                          
	|--Author      : ����ƽͷ��                                                     
	|--Date        : 2018-11-27             
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
#include "user.h" 
//#include "use_times.h" 
 /**
	* @Data    2019-01-16 19:28
	* @brief  ϵͳ��ʼ��
	* @param   void
	* @retval  void
	*/
		void SysInit(void)
		{ 
			SysInitCreate();
#ifdef DEBUG_BY_KEIL //keil���Գ�ʼ��
//			DebugByKeilInit();
#endif
		}
/*-----------------------------------file of end------------------------------*/
 



