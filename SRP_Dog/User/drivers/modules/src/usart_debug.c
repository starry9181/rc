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
  |--FileName    : usart_debug.c                                                
  |--Version     : v1.0                                                            
  |--Author      : ����ƽͷ��                                                       
  |--Date        : 2019-02-21               
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
#include "usart_debug.h" 
#ifdef DEBUG_BY_KEIL
#define USED_FLOAT 0x01 
#define USED_INT16 0x02
#define USED_UINT32 0x04 
#define IS_REUSED(_param) ( ((_param) == (USED_FLOAT)) || \
                            ((_param) == (USED_INT16)) || \
                            ((_param) == (USED_UINT32))  )
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart);
void usart2_niming_report(uint8_t fun,uint8_t*data,uint8_t len,\
													UART_HandleTypeDef* huart);
void PID_Debugfloat(float Target,float Real,UART_HandleTypeDef* huart);
void PID_Debugint16_t(int16_t Target,int16_t Real,UART_HandleTypeDef* huart);
void DebugUint32_t(UART_HandleTypeDef* huart,uint32_t data);
void NimingClassInit(void);
/* ----------------- �ⲿ���� -------------------- */
  extern UART_HandleTypeDef huart1;//����1
  extern UART_HandleTypeDef huart2;//����1
  extern UART_HandleTypeDef huart6;//����1
Niming_Class  Debug_t;
/* -------------------------------- begin 1 -------------------------------- */
	/**
	* @brief  ���ڷ���
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart)
{
    while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TXE)==0);//�ȴ���һ�η������  
    huart->Instance->DR=c;   
}

/*---------------------------------80�ַ�����-----------------------------------*/
  /**
  * @Data    2019-02-21 23:32
  * @brief   �ض���C�⺯�� printf ��USART2
  * @param   void
  * @retval  void
  */
	int fputc(int ch,FILE *f)  
  {   
    usart2_send_char((unsigned char)ch,&huart2);
    return (ch);    
  }
/* -------------------------------- begin 2 -------------------------------- */
	/**
	* @brief  �����ṹ���ʼ��
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void DebugClassInit(void)
{
	Debug_t.f=PID_Debugfloat;
	Debug_t.i16=PID_Debugint16_t;
	Debug_t.uint32 = DebugUint32_t;
	Debug_t.flag = 0;
}

/* -------------------------------- begin 3 -------------------------------- */
	/**
	* @brief     ��λ���������ݸ�������λ��
    * @param / 1��uint8_t ������   0xA0~0xAF
    *        / 2��������������ָ�� ���24���ֽ�
    *        / 3������������Ч�������ݳ��� 
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void usart2_niming_report(uint8_t fun,uint8_t*data,uint8_t len,UART_HandleTypeDef* huart)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;                                   //���28�ֽ�����
    send_buf[len+3]=0;                                  //У��������
    send_buf[0]=0X88;                                   //֡ͷ
    send_buf[1]=fun;                                    //������
    send_buf[2]=len;                                    //���ݳ���
    for(i=0;i<len;i++)send_buf[3+i]=data[i];            //��������
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];   //����У���
    for(i=0;i<len+4;i++)usart2_send_char(send_buf[i],huart);  //�������ݵ�����1
}

/* -------------------------------- begin 4 -------------------------------- */
   /**
    * @brief  ��������λ������PID����
    * @param / float Target_AngleĿ��Ƕ�
    *        / float Real_Angleʵ�ʽǶ�
    * @retval void
    **/
/* -------------------------------- end -------------------------------- */
void PID_Debugfloat(float Target,float Real,UART_HandleTypeDef* huart)
{
    uint8_t tbuf[8];
    unsigned char *p;
    /* -------- ʹ�ñ�־λ --------- */   
      SET_BIT(Debug_t.flag,USED_FLOAT);
    /* -------- ����Ƿ��ֵ��ã���ֹ��ӡ���ݻ��� --------- */
     assert_param(IS_REUSED(Debug_t.flag));
     p=(unsigned char *)&Target;
     tbuf[0]=(unsigned char)(*(p+3));
     tbuf[1]=(unsigned char)(*(p+2));
	 tbuf[2]=(unsigned char)(*(p+1));
	 tbuf[3]=(unsigned char)(*(p+0));

    p=(unsigned char *)&Real;
     tbuf[4]=(unsigned char)(*(p+3));
     tbuf[5]=(unsigned char)(*(p+2));
	 tbuf[6]=(unsigned char)(*(p+1));
	 tbuf[7]=(unsigned char)(*(p+0));
	  p=NULL;
    usart2_niming_report(0XA1,tbuf,8,huart);//�Զ���֡,0XA2
}
/* -------------------------------- begin 5 -------------------------------- */
   /**
    * @brief  ��������λ������PID����
    * @param / int16_t Target_AngleĿ��Ƕ�
    *        / int16_t Real_Angleʵ�ʽǶ�
    * @retval void
    **/
/* -------------------------------- end -------------------------------- */
void PID_Debugint16_t(int16_t Target,int16_t Real,UART_HandleTypeDef* huart)
{
  uint8_t tbuf[4];
  unsigned char *p;
  /* -------- ʹ�ñ�־λ --------- */   
  SET_BIT(Debug_t.flag,USED_INT16);
  /* -------- ����Ƿ��ֵ��ã���ֹ��ӡ���ݻ��� --------- */
  assert_param(IS_REUSED(Debug_t.flag));
  p=(unsigned char *)&Target;
  tbuf[0]=(unsigned char)(*(p+1));
  tbuf[1]=(unsigned char)(*(p+0));

  p=(unsigned char *)&Real;
  tbuf[2]=(unsigned char)(*(p+1));
  tbuf[3]=(unsigned char)(*(p+0));
  p=NULL;
  Debug_t.flag = USED_INT16;
  usart2_niming_report(0XA1,tbuf,4,huart);//�Զ���֡,0XA2
}
/**
* @Data    2019-02-21 18:15
* @brief   ����uint32_t�������ݸ�������λ��
* @param   void
* @retval  void
*/
void DebugUint32_t(UART_HandleTypeDef* huart,uint32_t data)
{
    uint8_t tbuf[4];
    unsigned char *p;
    /* -------- ʹ�ñ�־λ --------- */   
    SET_BIT(Debug_t.flag,USED_UINT32);
    /* -------- ����Ƿ��ֵ��ã���ֹ��ӡ���ݻ��� --------- */
    assert_param(IS_REUSED(Debug_t.flag));
    p=(unsigned char *)&data;
    tbuf[0]=(unsigned char)(*(p+3));
    tbuf[1]=(unsigned char)(*(p+2));
    tbuf[2]=(unsigned char)(*(p+1));
    tbuf[3]=(unsigned char)(*(p+0));
    p=NULL;
    Debug_t.flag = USED_UINT32;
    usart2_niming_report(0XA1,tbuf,4,huart);//�Զ���֡,0XA2
}
#endif
/*------------------------------------file of end-------------------------------*/



