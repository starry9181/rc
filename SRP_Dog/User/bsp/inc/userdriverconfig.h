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
  |--FileName    : userdriverconfig.h                                                
  |--Version     : v1.0                                                            
  |--Author      : ����ƽͷ��                                                       
  |--Date        : 2019-03-16               
  |--Libsupports : 
  |--Description : ���õײ�ӿ�                                                     
  |--FunctionList                                                       
  |-------1. ....                                                       
  |          <version>:                                                       
  |     <modify staff>:                                                       
  |             <data>:                                                       
  |      <description>:                                                        
  |-------2. ...                                                       
  |---------------------------------declaration of end----------------------------|
 **/
#ifndef __USERDRIVERCONFIG_H 
#define __USERDRIVERCONFIG_H 
/* -------- stm32 ��ѡ�� 1�ǿ�����0�ǿ���--------- */
// #define HAL_F1 0
#define HAL_F4 1
/* ----------------- �������ѡ�� 1�ǿ�����0�ǿ���-------------------- */
//#define RM_NEW_BOARD 0
//#define RM_OLD_BOARD 0
#define BINGE_BOARD 1 //���İ���
#if HAL_F1 
	// #include "stm32f1xx_hal.h"
	// #include <stdlib.h>
#elif HAL_F4
		#include "stm32f4xx_hal.h"
		#include "cmsis_os.h"
		#include <stdlib.h>
    #include <string.h>
    #include <stdio.h>
    #include <assert.h>
#endif 
/* -------------������ɫ�ӿ� ----------------- */
#define BLUE_GAME 0
#define RED_GAME  1

 //#define DEBUG_BY
/* ------------- �����ӿ�----------------- */
#define DISABLE_MOD 3
#define RC_MOD      2
#define AUTO_MOD    1
/* ----------------- led���ź궨�嶨�� -------------------- */
  #if RM_NEW_BOARD //�°����ź궨��
      #define LED_1 GPIO_PIN_1
      #define LED_2 GPIO_PIN_2
      #define LED_3 GPIO_PIN_3
      #define LED_4 GPIO_PIN_4
      #define LED_5 GPIO_PIN_5
      #define LED_6 GPIO_PIN_6
      #define LED_7 GPIO_PIN_7
      #define LED_8 GPIO_PIN_8
      #define LED_GPIO GPIOG
      #define LED_TOTAL 8 //led�ȸ���
      #define LED_ORIGIN_PIN 1  //��GPIO�������
      
      #define RED_LED GPIO_PIN_11//PE11
      #define GREEN_LED GPIO_PIN_14//PE14
      #define COLOR_LED_GPIO GPIOE
    /* ----------------- ���������ź궨�嶨�� -------------------- */  
      #define LASER GPIO_PIN_13//PG13
      #define LASER_GPIO GPIOG
    /* -----------------��Դ�������ź궨�嶨�� -------------------- */   
      #define POWER_1 GPIO_PIN_2//PH2
      #define POWER_2 GPIO_PIN_3//PH3
      #define POWER_3 GPIO_PIN_4//PH4
      #define POWER_4 GPIO_PIN_5//PH5
      #define POWER_GPIO GPIOH
    /* -----------------���������ź궨�嶨�� -------------------- */     
      #define BUZZER GPIO_PIN_6//PH6  TIM12_CH1
      #define BUZZER_GPIO GPIOH
    /* -------------- ���������ź궨�� ----------------- */
      #define SONIC            GPIO_PIN_1//PA1 Trig
      #define SONIC_LEFT_E     GPIO_PIN_2//PA2 Echo ���
      #define SONIC_RIGHT_E    GPIO_PIN_3//PA3 Echo �ұ�
      #define SONIC_GPIO       GPIOA
    /* -------------- ������⿪�����ź궨�� ----------------- */
    #define LASER_SWITCH       GPIO_PIN_5 //���
    #define LASER_SWITCH_BEF   GPIO_PIN_6//ǰ�ࣨ��������
    #define LASER_SWITCH_GPIO  GPIOI 
    /* ----------------- �������궨��ӿ� -------------------- */
     #define ENCODE_A    GPIO_PIN_10//������A��  
     #define ENCODE_B    GPIO_PIN_11//������B��   
     
    extern UART_HandleTypeDef huart1;//����1
    extern UART_HandleTypeDef huart2;//����2
    extern UART_HandleTypeDef huart6;
    extern UART_HandleTypeDef huart3;
    extern CAN_HandleTypeDef hcan1;
      extern CAN_HandleTypeDef hcan2;
    extern TIM_HandleTypeDef htim5;
    extern TIM_HandleTypeDef htim2;
		extern TIM_HandleTypeDef htim4;//Ħ���ֵ��
     #define ENCOER_TIM (&htim5)//��������ӿ�
     #define HCSR04_TIM (&htim2)//�������ӿ�
		 #define FRICTIONGEAR (&htim4)//Ħ���ֶ�ʱ�������ӿ�
		 #define FRICTIONGEAR_1 (TIM_CHANNEL_1)//Ħ����1PWMͨ��
		 #define FRICTIONGEAR_2 (TIM_CHANNEL_2)//Ħ����2PWMͨ��
		 #define FRICTIONGEAR_1_START_V (1000U)//Ħ����1�����ͺ�
		 #define FRICTIONGEAR_2_START_V (1000U)//Ħ����2�����ͺ�
		 #define GIMBAL_CAN (&hcan1)    //��̨�����can
     #define CHASSIS_CAN (&hcan1)    //��̨�����can
     #define  GY955_CAN (&hcan2)
     #define PC_DATA_UASRT (&huart6)//С�������ݽ��մ���
     #define COMMUNICAT    (&huart3)//����ϵͳ����
  #elif BINGE_BOARD
/* -----------------�����ⲿ���� -------------------- */  
   //������ʲô�Ͷ���ʲô��û���ö�ע�͵�
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
    extern UART_HandleTypeDef huart1;//����1
    extern UART_HandleTypeDef huart2;//����2
    extern UART_HandleTypeDef huart3;
    extern UART_HandleTypeDef huart7;
    extern UART_HandleTypeDef huart8;
	 	extern TIM_HandleTypeDef htim1;//Ħ���ֵ��
    extern TIM_HandleTypeDef htim3;
    extern TIM_HandleTypeDef htim2;
    extern TIM_HandleTypeDef htim7;
    extern TIM_HandleTypeDef htim12;
/* -----------------can�ӿں��װ -------------------- */  
  //������can���Ͷ���can��
    #define BSP_CAN1 (&hcan1)
    #define BSP_CAN2 (&hcan2)
/* -----------------ң�غ궨��ӿ� -------------------- */
     #define RC_UART (&huart1)//ң�ش��ں궨��
/* -------------���Ժ�oled��ʾ�궨��ӿ� ---------------- */
   /* ------ ���԰汾�ͷ��а汾ѡ�� --- */
    /*���а�������к궨��ע�͵�*/
   #define DEBUG_BY_KEIL
/* -----------------ȫ���궨��ӿ� -------------------- */
    //#define WHOLE_POSITION_UART 
/* ----------------- �������궨��ӿ� -------------------- */
     #define ENCODE_A    GPIO_PIN_6//������A��   
     #define ENCODE_B    GPIO_PIN_7//������B��  
     #define ENCODE_Z    GPIO_PIN_6//������B��  
     #define ENCODE_Z_GPIO GPIOE
     #define ENCODE_GPIO GPIOC
     #define ENCOER_TIM  (&htim3)//��������ӿ�
/* -------------- ���������ź궨�� ----------------- */
    #define SONIC            GPIO_PIN_1//PA2 Trig
    #define SONIC_LEFT_E     TIM_CHANNEL_3//PA0 Echo ���
    #define SONIC_RIGHT_E    TIM_CHANNEL_4//PA1 Echo �ұ�
    #define SONIC_LEFT_CHA    HAL_TIM_ACTIVE_CHANNEL_3
    #define SONIC_RIGHT_CHA    HAL_TIM_ACTIVE_CHANNEL_4
    #define SONIC_GPIO       GPIOA
    #define HCSR04_TIM (&htim2)//�������ӿ�
/* -------------- Ħ���ֺ궨�� ----------------- */
		 #define FRICTIONGEAR (&htim1)//Ħ���ֶ�ʱ�������ӿ�
		 #define FRICTIONGEAR_1 (TIM_CHANNEL_1)//Ħ����1PWMͨ��
		 #define FRICTIONGEAR_1_START_V (1000U)//Ħ����1�����ź�
     #define FRICTIONGEAR_PWM  GPIO_PIN_8
     #define FRICTIONGEAR_GPIO   GPIOA
/* -----------------���������ź궨�嶨�� -------------------- */     
      #define BUZZER GPIO_PIN_12//PH6  TIM12_CH1
      #define BUZZER_GPIO GPIOB
      #define BUZZER_TIM (&htim12)

		 #define GIMBAL_CAN (&hcan1)    //��̨�����can
     #define CHASSIS_CAN (&hcan2)    //��̨�����can
     #define  GYRO_CAN //(&hcan2)

     #define PC_DATA_UASRT (&huart3)//С�������ݽ��մ���
     #define COMMUNICAT    (&huart3)//����ϵͳ����
     #define HMI_USART     (&huart8)//����������
     #define RC_UART       (&huart1)//ң�ش��ں궨��
     #define DEBUG_UART (&huart2)

/* -------------- ������⿪�����ź궨�� ----------------- */
    #define LASER_SWITCH_BACK       GPIO_PIN_9 //���
    #define LASER_SWITCH_BEF   GPIO_PIN_11//ǰ�ࣨ��������
    #define LASER_SWITCH_GPIO  GPIOA 
/* ------��λ���غ�ӿ� ------- */  
		 #define LIM_SW_RIGHT         GPIO_PIN_9
		 #define LIM_SW_LEFT          GPIO_PIN_10
     #define LIM_SWITCH_GPIO  	  GPIOA
/* -------------- LED���ź궨�� ----------------- */
    #define LED_1 GPIO_PIN_9
    #define LED_2 GPIO_PIN_10
    #define LED_3 GPIO_PIN_11
    #define LED_4 GPIO_PIN_12
    #define LED_5 GPIO_PIN_13
    #define LED_6 GPIO_PIN_14
    #define LED_7 GPIO_PIN_15
    #define LED_GPIO GPIOE
    #define LED_TOTAL 7 //led�ȸ���
    #define LED_ORIGIN_PIN 9  //��GPIO�������
    
    #define RED_LED GPIO_PIN_4//PB4
    #define GREEN_LED GPIO_PIN_5//PB5
    #define COLOR_LED_GPIO GPIOB
/* -------------KEY���ź궨�� ----------------- */
    #define KEY_1 GPIO_PIN_7
    #define KEY_GPIO GPIOD
/* -------------֡��ͳ�ƺ궨��ӿ� ----------------- */
    #define _FPS_  //֡��ͳ�ƺ꿪��
    //���ö�ʱ��Ϊ1us����һ��  ptim = 90�� psc =89�� arr = 0xFFFF-1 �ʱ��Ϊ65ms
    
  #elif RM_OLD_BOARD
    #define LED_1 0
    #define LED_2 0
    #define LED_3 0
    #define LED_4 0
    #define LED_5 0
    #define LED_6 0
    #define LED_7 0
    #define LED_GPIO0G  0
    #define LED_TOTAL 0 //led�ȸ���
    #define LED_ORIGIN_PIN 0  //��GPIO�������
  #endif

#endif	// __USERDRIVERCONFIG_H

 /*------------------------------------file of end-------------------------------*/


