/**
	|-------------------------------- Copyright -----------------------------------|
	|                                                                              |
	|                        (C) Copyright 2019,����ƽͷ��,                         |
	|            1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                            All Rights Reserved                               |
	|                                                                              |
	|            By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)          |
	|                     https://github.com/GCUWildwolfteam                       |
	|------------------------------------------------------------------------------|
	|--FileName    : leds_tip.c                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-01-18               
	|--Libsupports : STM32CubeF1 Firmware Package V1.6.0(�ñ�Ŀ�����������)
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|---------------------------------declaration of end----------------------------|
 **/
#include "leds_tip.h" 
#if defined (LED_GPIO)
	/*---------------------------------80�ַ�����-----------------------------------*/
		/**
		* @Data    2019-01-18 11:38
		* @brief   ��˸��
		* @param   GPIO LED_GPIO (��ϸ˵����.h�ļ�)
		* @param   ledx led���ź궨��   (��ϸ˵����.h�ļ�)
		* @param   times ��˸����
		* @param   lag  ��˸ʱ���� ��λms
		* @retval  void
		*/
		void FlashingLed(GPIO_TypeDef* GPIO,uint16_t ledx,uint8_t times,uint32_t lag)
		{
		#if RM_OLD_BOARD
    	UNUSED(GPIO);
		#elif (BINGE_BOARD|RM_NEW_BOARD)
			uint8_t i = 0;
			for (i = 0; i < times*2;i++)
			{
					HAL_GPIO_TogglePin(GPIO, ledx);
          osDelay(lag);
			}
  	HAL_GPIO_WritePin(GPIO,ledx,GPIO_PIN_SET);
		#endif
		}
	/*---------------------------------80�ַ�����-----------------------------------*/
		/**
		* @Data    2019-01-18 14:16
		* @brief   ����LED��Ч (�ʺ�LED���������İ���)
		* @param   GPIO LED_GPIO (��ϸ˵����.h�ļ�)
		* @param 	 speed ��������ٶ�
		* @retval  void
		*/
		// void PulsatileLed(GPIO_TypeDef* GPIO,uint8_t speed)
		// {
		// 	uint8_t pulsatiles = 8;//��ԾLED����
		// 	uint8_t i = 0;
		// 	for (i = 0; i < pulsatiles;i++)
		// 	{

		// 	}
		// }
	/**
	* @Data    2019-01-18 14:25
	* @brief   ��������Ч
	* @param   GPIO LED_GPIO (��ϸ˵����baseclass.h�ļ�)
	* @param   lag  ��˸ʱ���� ��λms
	* @retval  void
	*/
	void ProgressBarLed(GPIO_TypeDef* GPIO,uint32_t lag)
	{
		#if RM_OLD_BOARD
		  UNUSED(GPIO);
		#elif (BINGE_BOARD|RM_NEW_BOARD)
		uint8_t i = 0;
		uint16_t tem_pin=0;
		for (i = 0; i < LED_TOTAL; i++)
		{
			tem_pin |= 1 << (i+LED_ORIGIN_PIN);
			HAL_GPIO_WritePin(GPIO, tem_pin, GPIO_PIN_RESET);
			osDelay(lag);
		}
		HAL_GPIO_WritePin(GPIO, tem_pin, GPIO_PIN_SET);
	#endif
	}
  /**
  * @Data    2019-03-16 01:26
  * @brief   ��ˮ������Ч
  * @param   void 
  * @retval  void
  */
//   void WaterfallLight(uint32_t times,uint8_t speed)
//   {
// #if RM_OLD_BOARD
// 		  UNUSED(GPIO);
// #elif (BINGE_BOARD|RM_NEW_BOARD)
// 		uint8_t i = 0;
// 		uint32_t tem_pin=0;
//     uint16_t tem_pin1;
//     uint16_t x ;
   
//     for( x= (uint16_t)(times/speed);x<1;x--)
//     {
// 	  	for (i = 0; i < LED_TOTAL; i++)
//       {
//         tem_pin = 7 << (i+LED_ORIGIN_PIN+7);
//         tem_pin1 = (uint16_t)(tem_pin >>8);
//         HAL_GPIO_WritePin(LED_GPIO, tem_pin1, GPIO_PIN_RESET);
//         osDelay(speed);
//       }
//     }

// 		HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
// #endif
//   }
  /**
  * @Data    2019-03-16 02:07
  * @brief   ��˸�����
  * @param   void
  * @retval  void
  */
  void WarningLed(uint8_t times,uint16_t lag)
  {
    uint16_t temp;
    temp = (LED_1|LED_2 |LED_3 |LED_4 |LED_5 |LED_6 |LED_7);
    FlashingLed(LED_GPIO,temp,times,lag);
  }
    /**
    * @Data    2019-03-16 02:24
    * @brief   ������Ч
    * @param   void
    * @retval  void
    */
    void SetpUpLED(void)
    {
      uint8_t i;
      HAL_GPIO_WritePin(LED_GPIO, i, GPIO_PIN_RESET);
    }
  /**
  * @Data    2019-03-16 02:24
  * @brief   �������м俿��Ч
  * @param   void
  * @retval  void
  */
   void SidesToMidLed(uint16_t speed)
   {
		#if RM_OLD_BOARD
		  UNUSED(GPIO);
		#elif (BINGE_BOARD|RM_NEW_BOARD)
		uint8_t i = 0;
		uint16_t tem_pin_left =0,tem_pin_right=0,tem_pin=0;
		for (i = 0; i < (uint8_t)(LED_TOTAL/2); i++)
		{
			tem_pin_right |= 1 << (i+LED_ORIGIN_PIN);
      tem_pin_left |= 1 << (LED_TOTAL+LED_ORIGIN_PIN - i-1);
      tem_pin |= (tem_pin_right| tem_pin_left);
			HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_RESET);
			osDelay(speed);
		}
    tem_pin |= 1 << (3+LED_ORIGIN_PIN);
    HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_RESET);
    osDelay(speed);
		HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
	#endif
  }
     /**
  * @Data    2019-03-16 02:24
  * @brief   �м������߿���Ч
  * @param   void
  * @retval  void
  */
   void MidToSidesLed(uint16_t speed)
   {
		#if RM_OLD_BOARD
		  UNUSED(GPIO);
		#elif (BINGE_BOARD|RM_NEW_BOARD)
		uint8_t i = 0;
		uint16_t tem_pin_left =0,tem_pin_right=0,tem_pin=0;
     tem_pin |= 1 << (3+LED_ORIGIN_PIN);
    HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_RESET);
    osDelay(speed);
		for (i = 0; i < (uint8_t)(LED_TOTAL/2); i++)
		{
			tem_pin_right |= 1 << (i+4+LED_ORIGIN_PIN);
      tem_pin_left |= 1 << (LED_ORIGIN_PIN+2 - i);
      tem_pin |= (tem_pin_right| tem_pin_left);
			HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_RESET);
			osDelay(speed);
		}
		HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
	#endif
  }
     /**
  * @Data    2019-03-16 02:24
  * @brief   �м������߿���Ч
  * @param   void
  * @retval  void
  */
   void PutOutMidToSidesLed(uint16_t speed)
   {
		#if RM_OLD_BOARD
		  UNUSED(GPIO);
		#elif (BINGE_BOARD|RM_NEW_BOARD)
		uint8_t i = 0;
		uint16_t tem_pin_left =0,tem_pin_right=0,tem_pin=0;
     HAL_GPIO_WritePin(LED_GPIO, 0xFE00, GPIO_PIN_RESET);
       osDelay(speed);
     tem_pin |= 1 << (3+LED_ORIGIN_PIN);
    HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
    osDelay(speed);
		for (i = 0; i < (uint8_t)(LED_TOTAL/2); i++)
		{
			tem_pin_right |= 1 << (i+4+LED_ORIGIN_PIN);
      tem_pin_left |= 1 << (LED_ORIGIN_PIN+2 - i);
      tem_pin |= (tem_pin_right| tem_pin_left);
			HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
			osDelay(speed);
		}
		HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
	#endif
  }
  /**
  * @Data    2019-03-16 02:24
  * @brief   �������м俿��Ч
  * @param   void
  * @retval  void
  */
   void PutOutSidesToMidLed(uint16_t speed)
   {
		#if RM_OLD_BOARD
		  UNUSED(GPIO);
		#elif (BINGE_BOARD|RM_NEW_BOARD)
		uint8_t i = 0;
		uint16_t tem_pin_left =0,tem_pin_right=0,tem_pin=0;
    HAL_GPIO_WritePin(LED_GPIO, 0xFE00, GPIO_PIN_RESET);
     osDelay(speed);
		for (i = 0; i < (uint8_t)(LED_TOTAL/2); i++)
		{
			tem_pin_right |= 1 << (i+LED_ORIGIN_PIN);
      tem_pin_left |= 1 << (LED_TOTAL+LED_ORIGIN_PIN - i-1);
      tem_pin |= (tem_pin_right| tem_pin_left);
			HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
			osDelay(speed);
		}
    tem_pin |= 1 << (3+LED_ORIGIN_PIN);
    HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
   // osDelay(speed);
		//HAL_GPIO_WritePin(LED_GPIO, tem_pin, GPIO_PIN_SET);
	#endif
  }
#endif
/*------------------------------------file of end-------------------------------*/


