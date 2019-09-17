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
	|--FileName    : debug_by_keil.c                                              
	|--Version     : v1.0                                                          
	|--Author      : ����ƽͷ��                                                     
	|--Date        : 2019-02-23             
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
#include "debug_by_keil.h" 
#ifdef DEBUG_BY_KEIL
/* -------- ��ӡ����״̬ --------- */
//#define PRINTFTASKSTATUS 
 extern UART_HandleTypeDef huart2;//����1
 extern UART_HandleTypeDef huart7;
extern TIM_HandleTypeDef htim2;
extern	 pcDataStruct pc_t;//С��������;
 uint32_t runtimeCounter;
debugByKeilStruct pdebug_t;
osThreadId startDebugByKeilTaskHandle; 
void StartDebugByKeilTask(void const *argument);
/* ======================== ��ʱ���Դ������ of begin ======================== */
	uint8_t aaa=0;
/* ======================== ��ʱ���Դ������ of end ======================== */



/**
  * @Data    2019-02-23 15:46
  * @brief   ��ȡ�ֲ������ĵ�ַ������keilӲ������
  * @param   void
  * @retval  void
  */
  void DebugByKeilInit(void)
  {
    /* -------- ��������ʱ�䶨ʱ�� --------- */
//    HAL_TIM_Base_Start_IT(&htim10);
    /* -------- keilӲ����������ʾ���� --------- */
//     osThreadDef(debugByKeilTask, StartDebugByKeilTask, osPriorityHigh, 0,DEBUG_HEAP_SIZE);
//     startDebugByKeilTaskHandle = osThreadCreate(osThread(debugByKeilTask), NULL); 
////    pdebug_t.d_rc= GetRcStructAddr();
//   pdebug_t.d_pchassis_t = GetChassisStructAddr();

//			 DebugClassInit();   

    
  }
/**
  * @Data    2019-02-23 15:46
  * @brief   keilӲ�����������Ӻ���
  * @param   void
  * @retval  void
  */
  int16_t gggfffdg =0;
void StartDebugByKeilTask(void const *argument)
{
#ifdef PRINTFTASKSTATUS
     uint8_t pcWriteBuffer[500];
#endif
		for(;;)
		{
#ifdef PRINTFTASKSTATUS
      taskENTER_CRITICAL();
        memset(pcWriteBuffer, 0, 500);
        vTaskList((char *)&pcWriteBuffer);
        printf("�������ڴ��С = %d\r\n",configTOTAL_HEAP_SIZE);
        printf("��������     ����״̬  ���ȼ�  ʣ���ջ  �������\r\n");
        printf("-------------------------------------------------\r\n");
        printf("%s  \r\n", pcWriteBuffer); 
        printf("-------------------------------------------------\r\n");
        printf("B:����    R:����    D:ɾ��    S:��ͣ\r\n");
        printf("-------------------------------------------------\r\n");
        memset(pcWriteBuffer, 0, 500);
        printf("��������\t���м���\tʹ����\r\n");
        printf("-------------------------------------------------\r\n");
        vTaskGetRunTimeStats((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
        printf("-------------------------------------------------\r\n");
     taskEXIT_CRITICAL();
        osDelay(100);      
#endif
//     if(GetFps(pc_t.fps) ==0)
//   {
//       __HAL_TIM_SetCompare(BUZZER_TIM,FRICTIONGEAR_1,10000);
//     }  
//    __HAL_TIM_SetCompare(BUZZER_TIM,FRICTIONGEAR_1,0);
 osDelay(100); 
		}
}
/**
* @Data    2019-02-23 15:46
* @brief   �û���ʱ���ص�����
* @param   void
* @retval  void
*/
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM10)
  {
    runtimeCounter++;
  }
}
void configureTimerForRunTimeStats(void)
{
  runtimeCounter = 0;
}

unsigned long getRunTimeCounterValue(void)
{
return runtimeCounter;
}
/* =========================== ���Դ����� of begin =========================== */

/* =========================== ���Դ����� of end =========================== */
#endif
/*-----------------------------------file of end------------------------------*/


