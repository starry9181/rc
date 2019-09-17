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
	|--FileName    : fps.c                                              
	|--Version     : v1.0                                                          
	|--Author      : ����ƽͷ��                                                     
	|--Date        : 2019-04-23             
	|--Libsupports : 
	|--Description :1.��ϵͳ��ʼ��ʱ����FpsUserTimeInit()����֡����ʱ��ʹ��֡��ͳ����                   
  |--             2.����fps_t���� ����Ҫͳ��֡��ģ���ʼ��ʱ����SetFpsAddress(); ��
	|--									��֡�����͵�ַ
  |--             3.�ڽ��պ�������Fps();ͳ��֡��
	|--FunctionList                                                     
	|-------1. ....                                                     
	|          <version>:                                                     
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#include "fps.h" 
#define FPS_POOL_SIZE 30
#define AR_SIZE      2
#define FPS_POOL     0
#define COUNTER_POOL 1
 /* -------------- �����ʱ�� ----------------- */       
osTimerId fpsTimerHandle;
void FpsCallback(void const * argument);
static uint32_t *s_fps_addr[FPS_POOL_SIZE];
static uint32_t s_counters[FPS_POOL_SIZE];
#define OFFLINE_MAX_TIME   200 //�������ʱ��0.5s
#define FPS_BASE_TIME_CEO   (1000/OFFLINE_MAX_TIME)  //֡��ʱ��ϵ�� 
/**
	* @Data    2019-04-23 06:00
	* @brief   ֡����ʱ����
	* @param   void
	* @retval  SYS_Status
	*/
	SYS_Status FpsUserTimeInit(void)
	{
		osStatus timerresult = osOK;
    osTimerDef(FpsTimer, FpsCallback);
    fpsTimerHandle = osTimerCreate(osTimer(FpsTimer), osTimerPeriodic, NULL);
    timerresult = osTimerStart(fpsTimerHandle,OFFLINE_MAX_TIME);
    if(timerresult !=osOK)
    {
      return SYS_ERROR;
    }
		return SYS_OK;
	}
/**
	* @Data    2019-04-23 05:50
	* @brief  ֡��ͳ�� 
	* @param   fps_t���͵�ַ //��fps.h
	* @retval  void
	*/
void Fps(uint32_t fps[])
{
   s_counters[fps[FPS_ADDR]]++;
}
/**
	* @Data    2019-04-23 06:10
	* @brief   ����֡������ַ
	* @param   fps_t���͵�ַ //��fps.h
	* @retval  SYS_Status
	*/
	SYS_Status SetFpsAddress(uint32_t fps[])
	{
     for(uint8_t i=0;i<FPS_POOL_SIZE;i++)
    {
      if( s_fps_addr[i]==NULL)
      {
        s_fps_addr[i] = &fps[FPS];
         fps[FPS_ADDR] = i; 
        return SYS_OK;
      }
    }
     return SYS_ERROR;
	}

/**
	* @Data    2019-04-23 05:56
	* @brief   ֡�������ʱ���ص�����
	* @param   void
	* @retval  void
	*/
void FpsCallback(void const * argument)
{
  for(uint8_t i=0;i<FPS_POOL_SIZE;i++)
  {
    if(s_fps_addr[i] !=NULL)
    {
      *s_fps_addr[i]= (uint32_t)(s_counters[i] * FPS_BASE_TIME_CEO);
      s_counters[i] = 0;
    }
  }
}

/*-----------------------------------file of end------------------------------*/


