/**
	|------------------------------ Copyright -----------------------------------|
	|                                                                            |
	|                       (C) Copyright 2019,����ƽͷ��,                        |
	|          1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                            All Rights Reserved                             |
	|                                         -                                  |
	|          By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)          |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : bsp_usart.c                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-01-15               
	|--Libsupports : 
	|--Description :1.cubemx���ô��ں���Ӧ��DMA��������Ӧ���ж�
  |               2.����UsartAndDMAInit��ʼ��������Ӧ������
                  3.UserUsartCallback ��stm32f4xx_it.c �е���Ӧ�жϺ����е���
                  4.UserUsartQueueRX ����Ҫ�������ݵ�ģ����ý���
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|------------------------------declaration of end----------------------------|
 **/
#include "bsp_usart.h"
#define USART_NUM 9
/* ----------------- �ṹ���ַ�б� -------------------- */
usartDataStrcut *pusart_t[USART_NUM] = {NULL};
/**
	* @Data    2019-01-15 15:31
	* @brief   ���ڳ�ʼ��
	* @param	 usartx(x=1~8)
	* @param   uint8_t frame_size һ֡�ĳ���
	* @param   on_off   DISABLE = 0U, ENABLE = 1U //��ʼ������
	* @retval  SYS_Status
	*/
SYS_Status UsartAndDMAInit(UART_HandleTypeDef *huartx,uint8_t frame_size\
																															,uint8_t on_off)
{
	usartDataStrcut *addr;
	/* -------- ����ӿ����ݻ�ȡ --------- */
	if(AllocateUsartxSpace(huartx) != HAL_OK)//�û����ڽṹ��ռ����
	{
		free(addr);
		return SYS_ERROR;
	}
	addr = GetUsartAddr(huartx); //��ȡ��Ӧ�û����ڽṹ���ַ
  if(addr == NULL)
    return SYS_ERROR;
	addr->usart_queue = NULL; 
	addr->rx_buff_size = frame_size; //�õ�һ֡�ĳ���
	addr->rx_on_off = on_off;//���տ���
	/* ------ ������ջ����ַ�ռ� ------- */
	addr->rx_buff_data = (uint8_t *)malloc(addr->rx_buff_size * sizeof(uint8_t));
  if(addr->rx_buff_data == NULL)
    return SYS_ERROR; 
	if(UsartQueueCreate(addr,5,addr->rx_buff_size) != HAL_OK) //��������
	{
		free(addr->rx_buff_data);
		free(addr);
		return SYS_ERROR;
	}
  	/* -------- ʹ�� --------- */
  HAL_UART_Receive_DMA(huartx,addr->rx_buff_data,addr->rx_buff_size);
	__HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);					 //ʹ�ܴ����ж�
	return SYS_OK;
}
/**
	* @Data    2019-01-15 18:58
	* @brief   �����û��жϻص�����
	* @param   huartx(x=1~8) �û����ڽṹ���ַ
	* @retval  void
	*/
	void UserUsartCallback(UART_HandleTypeDef *huartx)
	{
		NoLenRXforUsart(huartx);
	}
/**
	* @Data    2019-01-15 19:07
	* @brief   �����ֽڽ���
	* @param   huartx��1~8�� �û����ڽṹ���ַ
	* @retval SYS_Status
	*/
SYS_Status NoLenRXforUsart(UART_HandleTypeDef *huartx)
{
  uint32_t temp; 
  usartDataStrcut *addr = NULL;
  floatToUnion p;
  addr = GetUsartAddr(huartx); //��ȡ��Ӧ�û����ڽṹ���ַ
  if(addr == NULL || huartx ==NULL)
    return SYS_ERROR;
  if((__HAL_UART_GET_FLAG(huartx,UART_FLAG_IDLE) != RESET))  
  {
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    HAL_UART_DMAStop(huartx);  
    temp = huartx->hdmarx->Instance->NDTR; 
    addr->datalen = addr->rx_buff_size-temp;
    p.u_16[0] = addr->datalen;
    addr->rx_buff_data[DATA_LEN_BYTE_HIGH_8] = p.u_8[DATA_LEN_BYTE_HIGH_8];
    addr->rx_buff_data[DATA_LEN_BYTE_LOW_8]  = p.u_8[DATA_LEN_BYTE_LOW_8];
    xQueueSendToBackFromISR(addr->usart_queue, addr->rx_buff_data,0);
    memset(addr->rx_buff_data,0,addr->rx_buff_size);
    HAL_UART_Receive_DMA(huartx,(addr->rx_buff_data + RX_HEAD_ADDR),addr->rx_buff_size);
  }
  return SYS_OK;
}
/**
	* @Data    2019-01-16 10:24
	* @brief   ���д���
	* @param   void
	* @retval  SYS_Status
	*/
	SYS_Status UsartQueueCreate(usartDataStrcut *usartx,uint8_t len,uint8_t deep)
	{
    if(usartx == NULL)
      return SYS_ERROR; 
		usartx->usart_queue = xQueueCreate(len,deep);//�������len����21����
		if(usartx->usart_queue == NULL)
			return SYS_ERROR;
		return SYS_OK;
	}
/**
	* @Data    2019-01-16 10:54
	* @brief   ������Ӧ�����������ݵĿռ�
	* @param   huartx��1~8��
	* @retval  SYS_Status
	*/
SYS_Status AllocateUsartxSpace(UART_HandleTypeDef *huartx)
{
	if (huartx->Instance == USART1) 
	{
		pusart_t[1]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[1] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
	else if (huartx->Instance == USART2) 
	{
		pusart_t[2]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[2]== NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
	else if (huartx->Instance == USART3) 
	{
		pusart_t[3]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[3] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
  else if (huartx->Instance == UART4) 
	{
		pusart_t[4]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[4] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
  else if (huartx->Instance == UART5) 
	{
		pusart_t[5]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[5] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
	else if (huartx->Instance == USART6) 
	{
		pusart_t[6]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[6] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	}
  else if(huartx->Instance == UART7)
  {
    pusart_t[7]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[7] == NULL)
		return SYS_ERROR;
		return SYS_OK;
  }
   else if(huartx->Instance == UART8)
  {
    pusart_t[8]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[8] == NULL)
		return SYS_ERROR;
		return SYS_OK;
  }
	else	return SYS_ERROR;
}
/**
	* @Data    2019-01-16 11:08
	* @brief   �Զ��б𴮿����ͻ�ȡ��Ӧ�û����ڽṹ���ַ
	* @param   huartx��1~8��
	* @retval  usartDataStrcut* �û����ڽṹ��ָ��
	*/
	usartDataStrcut* GetUsartAddr(UART_HandleTypeDef *huartx)
	{
		if (huartx->Instance == USART1) 
			return pusart_t[1];
		else if (huartx->Instance == USART2) 
			return pusart_t[2];
		else if (huartx->Instance == USART3) 
			return pusart_t[3];
		else if (huartx->Instance == UART4) 
			return pusart_t[4];
		else if (huartx->Instance == UART5)
			return pusart_t[5];
		else if (huartx->Instance == USART6)  
			return pusart_t[6];
    else if(huartx->Instance == UART7) 
			return pusart_t[7];
    else if(huartx->Instance == UART8) 
			return pusart_t[8];
		else	return NULL;
	}

/**
	* @Data    2019-01-16 15:22
	* @brief   ���н���
	* @param   huartx��1~8��
	* @param 	 pvBuffer �������ݵ�ַ
	* @retval  һ֡���ݽ��յ���ʵ���ȣ�������ʱ�����ؿ�EMPTY
	*/
	uint16_t UserUsartQueueRX(UART_HandleTypeDef *huartx,uint8_t* pvBuffer)
	{
    portBASE_TYPE xStatus;
    usartDataStrcut *addr = NULL;
    addr = GetUsartAddr(huartx);
    if(addr == NULL || pvBuffer == NULL)
      return DATA_EMPTY;
    xStatus = xQueueReceive(addr->usart_queue, pvBuffer, 0);
    if(pdFAIL != xStatus)
      return (LEFT_SH(pvBuffer[DATA_LEN_BYTE_HIGH_8],8) |pvBuffer[DATA_LEN_BYTE_LOW_8]);
    return DATA_EMPTY;
	}
/*-----------------------------------file of end------------------------------*/
