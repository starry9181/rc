/**
	|-------------------------------- Copyright ---------------------------------|
	|                                                                            |
	|                        (C) Copyright 2019,����ƽͷ��,                       |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                          All Rights Reserved                               |
	|                                                                            |
	|           By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : bsp_can.c                                                
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
	|--------------------------------declaration of end--------------------------|
 **/
#include "bsp_can.h" 
/* ----------------- �ṹ���ַ�б� -------------------- */
canDataStrcut *pcan_t[3] = {NULL};
/**
	* @Data    2019-01-26 12:27
	* @brief   can���ó�ʼ��
	* @param   void
	* @retval SYS_Status
	*/
SYS_Status UserCanConfig(void)   
{
  CAN_HandleTypeDef *hcanx;
  /*���can1û�����ã�������can1*/
  if(pcan_t[1] == NULL && BSP_CAN1 != NULL)
    hcanx = BSP_CAN1;
  /*���can1�ѱ����ã�������can2*/
  else if(pcan_t[1] != NULL && pcan_t[2] == NULL && BSP_CAN2 != NULL)
    hcanx = BSP_CAN2;
  else return SYS_ERROR; 
	if(AllocateCanxSpace(hcanx) != HAL_OK)//�û�can�ṹ��ռ����
    return SYS_ERROR;
	else if(CanFilterInit(hcanx) != HAL_OK)//��������
    return SYS_ERROR;
	else if(CanTxInit(hcanx) != HAL_OK)//��������
		return SYS_ERROR;
	else if(CanRxInit(hcanx) != HAL_OK)//��������
		return SYS_ERROR;
  else
  {
    HAL_CAN_Start(hcanx);//����
    HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING); //�����ж�
    return SYS_OK;   
  }

}
/**
	* @Data    2019-01-26 12:28
	* @brief   can��������
	* @param   CAN_HandleTypeDef *hcanx(x=1,2)
	* @retval   HAL Status
	*/
	
SYS_Status CanFilterInit(CAN_HandleTypeDef* hcanx)
{
	canDataStrcut *addr;
	addr = GetCanAddr(hcanx);
	addr->filter.FilterIdHigh = 0x0000;//Ҫ���˵�ID��λ
	addr->filter.FilterIdLow = 0x0000;//Ҫ���˵�ID��λ
	addr->filter.FilterMaskIdHigh = 0x0000;//��������16λÿλ����ƥ��
	addr->filter.FilterMaskIdLow = 0x0000;//��������16λÿλ����ƥ��
	addr->filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//��������������FIFO 0
	addr->filter.FilterScale = CAN_FILTERSCALE_32BIT;
	addr->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	addr->filter.FilterActivation = ENABLE;//ʹ�ܹ�����
  if(hcanx->Instance ==CAN1)
  {
    addr->filter.FilterBank = 0;
    addr->filter.SlaveStartFilterBank = 0;
  }
  else if(hcanx->Instance ==CAN2)
  {
    addr->filter.FilterBank = 14;
    addr->filter.SlaveStartFilterBank = 14;
  }
  if(HAL_CAN_ConfigFilter(hcanx, &addr->filter) != HAL_ERROR)//ʹ��ɸѡ��
    return SYS_ERROR;
	else return SYS_OK; 
}
/**
	* @Data    2019-01-26 12:28
	* @brief   can ���ͽṹ���ʼ��
	* @param   CanTxMsgTypeDef* TxMsg
	* @retval  HAL Status
	*/
SYS_Status CanTxInit(CAN_HandleTypeDef* hcanx)
{
	canDataStrcut *addr;
	addr = GetCanAddr(hcanx);//��ȡ��Ӧ�û�can�ṹ���ַ
	if(addr == NULL)
	{
		return SYS_ERROR;
	}
	addr->txMsg.StdId = 0x000; //����id
	addr->txMsg.IDE = CAN_ID_STD; //ѡ���׼id
	addr->txMsg.RTR = CAN_RTR_DATA; //0Ϊ����֡��1ΪԶ��֡
	addr->txMsg.DLC = 8; //�������ݳ���Ϊ8���ֽ�
	return SYS_OK;
}
/**
	* @Data    2019-01-26 12:29
	* @brief   can ���ܽṹ���ʼ��
	* @param   CanRxMsgTypeDef* RxMsg
	* @retval  void
	*/
SYS_Status CanRxInit(CAN_HandleTypeDef* hcanx)
{
		canDataStrcut *addr;
	  addr = GetCanAddr(hcanx); //��ȡ��Ӧ�û�can�ṹ���ַ
		if(addr == NULL)
		{
			return SYS_ERROR;
		}
    addr->rxMsg.StdId=0x00;
    addr->rxMsg.ExtId=0x00;
    addr->rxMsg.DLC=0x00;
    addr->rxdata[0] = 0x00; //��������λ������
    addr->rxdata[1] = 0x00;
    addr->rxdata[2] = 0x00;
    addr->rxdata[3] = 0x00;
    addr->rxdata[4] = 0x00;
    addr->rxdata[5] = 0x00;
    addr->rxdata[6] = 0x00;
    addr->rxdata[7] = 0x00;
		return SYS_OK;
}
/**
	* @Data    2019-01-16 10:54
	* @brief   ������Ӧcan�������ݵĿռ�
	* @param   hcanx ��x=1,2��
	* @retval  HAL Status
	*/
	SYS_Status AllocateCanxSpace(CAN_HandleTypeDef *hcanx)
	{
		if (hcanx->Instance == CAN1) 
		{
			pcan_t[1]	= (struct canDataStrcut*)malloc(sizeof(struct canDataStrcut));
			if(pcan_t[1] == NULL)
			return SYS_ERROR;
			return SYS_OK;
		} 
		else if (hcanx->Instance == CAN2) 
		{
			pcan_t[2]	= (struct canDataStrcut*)malloc(sizeof(struct canDataStrcut));
			if(pcan_t[2]== NULL)
			return SYS_ERROR;
			return SYS_OK;
		}
		else	return SYS_ERROR;
	}
/**
	* @Data    2019-01-16 11:08
	* @brief   �Զ��б�can ���ͻ�ȡ��Ӧ�û�can�ṹ���ַ
	* @param   hcanx ��x=1,2��
	* @retval  canDataStrcut* �û����ڽṹ��ָ��
	*/
	canDataStrcut* GetCanAddr(CAN_HandleTypeDef *hcanx)
	{
		if(hcanx->Instance == CAN1)
		{
			return pcan_t[1];
		} 
		else if(hcanx->Instance == CAN2)  
		{
			return pcan_t[2];
		} 
		else	return NULL;
	}
/**
	* @Data    2019-01-19 00:58
	* @brief   can���ܻص�����
	* @param   void
	* @retval  void
	*/
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		canDataStrcut *addr;
	  addr = GetCanAddr(hcan); //��ȡ��Ӧ�û�can�ṹ���ַ
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&addr->rxMsg,addr->rxdata);
    if(hcan->Instance ==CAN1)
      can1_rx(addr->rxMsg.StdId,addr->rxdata);
    else if(hcan->Instance ==CAN2)
      can2_rx(addr->rxMsg.StdId,addr->rxdata);
	}
/**
	* @Data    2019-02-15 13:40
	* @brief   can���ͺ���
	* @param   void
	* @retval  void
	*/
 SYS_Status CanTxMsg(CAN_HandleTypeDef* hcanx,int id,uint8_t *message)
 {
  canDataStrcut *addr = NULL;
  addr = GetCanAddr(hcanx); //���Ҷ�Ӧ��˽�нṹ��
  if(addr == NULL)	
    return SYS_ERROR;														      										
  addr->txMsg.StdId = id; //����id
  addr->txMsg.IDE = CAN_ID_STD; //ѡ���׼id
  addr->txMsg.RTR = CAN_RTR_DATA; //0Ϊ����֡��1ΪԶ��֡
  addr->txMsg.DLC = 8; //�������ݳ���Ϊ8���ֽ�
  HAL_CAN_AddTxMessage(hcanx,&addr->txMsg,message,(uint32_t*)CAN_TX_MAILBOX0);
  return SYS_ERROR;
}
/**
	* @Data    2019-02-15 13:40
	* @brief   can1�������ݴ���ص�
	* @param   uint32_t id can����id
  * @param   uint8_t *data ���ջ����ַ
	* @retval  void
	*/		       
__weak void can1_rx(uint32_t id,uint8_t *data)
{
   (void)id;
   (void)data;
}
/**
	* @Data    2019-02-15 13:40
	* @brief   can2�������ݴ���ص�
	* @param   uint32_t id can����id
  * @param   uint8_t *data ���ջ����ַ
	* @retval  void
	*/	
__weak void can2_rx(uint32_t id,uint8_t *data)
{
   (void)id;
   (void)data;  
}
/*-----------------------------------file of end------------------------------*/


