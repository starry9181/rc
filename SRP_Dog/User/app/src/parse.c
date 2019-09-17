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
	|--FileName    : parse.c                                              
	|--Version     : v1.0                                                          
	|--Author      : ����ƽͷ��                                                     
	|--Date        : 2019-02-02             
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
#include "parse.h" 
/* -------------- �ṹ�� ----------------- */
 dbusStruct dbus_t; //��ң��
 //pcDataStruct pc_t; //openmv����
 //hmiStrct hmi_t;    //������
/* -------------- �ⲿ���� ----------------- */
extern osThreadId startParseTaskHandle;
extern UART_HandleTypeDef huart1;//����1
extern uint8_t can1_rx[12];
//extern uint32_t lenjuu;
	/**
	* @Data    2019-02-14 14:36
	* @brief   ���ݽ��������ʼ��
	* @param   void
	* @retval  void
	*/
void ParseInit(void)
{
  DJIDbusInit(&dbus_t,&huart1);//��ң�س�ʼ��
  //PcDataRxInit(&pc_t);         //openmv���ݽ��ճ�ʼ��
  //HmiUsartInit(&hmi_t);        //��������ʼ��
  /* -------- ����ȴ�����ϵͳ��ʼ�� --------- */
	vTaskSuspend(startParseTaskHandle);
}

	/**
	* @Data    2019-02-14 14:36
	* @brief   ���ݽ���
	* @param   void
	* @retval  void
	*/
	void ParseData(void)
	{
    DbusParseData(&dbus_t);//ң�����ݽ���
    //Pc_ParseData(&pc_t);//openmv���ݽ���
    //HmiUsartParseData(&hmi_t);
	}
 /*
	* @Data    2019-02-24 11:59
	* @brief   ��ȡң�ع����ַ
	* @param   void
	* @retval  void
	*/
	const dbusStruct* GetRcStructAddr(void)
	{
		return &dbus_t;
	}
	/**
	* @Data    2019-03-13 15:30
	* @brief   
	* @param   void
	* @retval  void
	*/
	uint32_t GetRcStatus(void)
	{
		return (dbus_t.status);
	}
  /**
	* @Data    2019-03-13 15:30
	* @brief   
	* @param   void
	* @retval  void
	*/
//	uint32_t GetPcDataStatus(void)
//	{
//		//return (pc_t.status);
//	}
  /**
  * @Data    2019-03-21 01:09
  * @brief   ��ȡС���Թ����ַ
  * @param   void
  * @retval  void
  */
//  const pcDataStruct* GetPcDataStructAddr(void)
//  {
//   // return (&pc_t);
//  }
//void can_rx(uint32_t id,uint8_t *data)
//  {
//    ChassisParseDate(id,data);
//    GimbalParseDate(id,data);
//  }
  void can_rx(uint32_t id,uint8_t *data)
  {

  }
void can2_rx(uint32_t id,uint8_t *data)
{
      //ChassisParseDate(id,data);
}
/*-----------------------------------file of end------------------------------*/


