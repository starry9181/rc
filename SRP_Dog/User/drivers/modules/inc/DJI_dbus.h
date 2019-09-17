/**
	|-------------------------------- Copyright ---------------------------------|
	|                                                                            |
	|                      (C) Copyright 2019,����ƽͷ��,                         |
	|          1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                           All Rights Reserved                              |
	|                                                                            |
	|           By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)         |
	|                     https://github.com/GCUWildwolfteam                     |
	|----------------------------------------------------------------------------|
	|--FileName    : DJI_dbus.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-01-15               
	|--Libsupports : 
	|--Description : 1.��Ӵ��������ļ� bsp_usart.h��bsp_usart.c
  |                2.���֡��ͳ���ļ� fps.h��fps.c   
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-------------------------------declaration of end---------------------------|
 **/
#ifndef __DJI_DBUS_H 
#define __DJI_DBUS_H 
	#include "bsp_usart.h"
  #include "fps.h" 
#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001
	typedef struct dbusStruct
	{
		int16_t ch1;	//each ch value from -660 -- +660
		int16_t ch2;
		int16_t ch3;
		int16_t ch4;
		
		uint8_t switch_left;	//3 value
		uint8_t switch_right;
    int16_t thumbwheel;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
		
			uint8_t press_left;
			uint8_t press_right;
					
			uint8_t jumppress_left;
			uint8_t jumppress_right;
			
		}mouse;
		struct 
		{
	/*****************************************************************************
	 * ����ͨ��:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
	 *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
	*****************************************************************************/
			uint16_t key_code;              //ԭʼ��ֵ
					uint16_t jumpkey_code;          //�����ļ�ֵ
		}keyBoard;
     uint32_t status;//״̬
		uint32_t state_flag;//��־λ
		uint8_t a_frame_len;//һ֡���ݳ���
    fps_t fps;//֡�ʼ���
		UART_HandleTypeDef *huartx; //��ȡ���ڵ�ַָ��
	} dbusStruct;
/*�Ѵ˽ṹ��ʵ������Ȼ�����Ӧ�ı�����ַ���ݸ���Ӧ������ͨ����û�õ�����NULL*/
  typedef struct redifRcStruct
  {
    int16_t* ch1;	//each ch value from -660 -- +660
		int16_t* ch2;
		int16_t* ch3;
		int16_t* ch4;
    uint8_t* switch_left;	//3 value
		uint8_t* switch_right;
    int16_t* thumbwheel;
    const dbusStruct* pRc_t;
  }redifRcStruct;//�����ض���  //��¼��ÿ������һ���ṹ�壬�������źͶ��尴������
HAL_StatusTypeDef DJIDbusInit(dbusStruct *dbs, UART_HandleTypeDef *huartx);
void DbusParseData(dbusStruct *dbs);
int16_t DbusAntiShake(int16_t data,int16_t range);
SYS_Status DefineRcKey(redifRcStruct* pRdK_t);
#endif // __DJI_DBUS_H

/*----------------------------------file of end-------------------------------*/
