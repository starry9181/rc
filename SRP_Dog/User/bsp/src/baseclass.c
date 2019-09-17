/**
	|----------------------------- Copyright ------------------------------------|
	|                                                                            |
	|                        (C) Copyright 2018,����ƽͷ��,                       |
	|        1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China         |
	|                         All Rights Reserved                                |
	|                                                                            |
	|         By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)           |
	|                  https://github.com/GCUWildwolfteam                        |
	|----------------------------------------------------------------------------|
	|--FileName    : baseclass.c                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2018-12-30               
	|--Libsupports : 
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|------------------------------declaration of end----------------------------|
 **/
#include "baseclass.h" 
	/**
	* @Data    2018-12-30 17:23
	* @brief   ��ȡ��������ָ��
	* @param   void* 
	* @retval  void*
	*/
	void* Get_Peripheral_type(void* Peripheral)
	{
		return Peripheral;
	}
	/**
		* @Data    2019-02-14 15:09
		* @brief   ���ֽ�����ת���ֽ�����,��λ��ǰ
		* @param   uint32_t �ĸ��ֽ�
		* @param	 uint8_t *���ֽ�ָ�� 
		* @retval  void
		*/
		void MultibyteToByle(uint32_t data,uint8_t *pdata)
		{
			unsigned char *p;
			p = (unsigned char *)&data;
			*pdata = (unsigned char)*(p + 3);//�߰�λ
			*(pdata+1)= (unsigned char)*(p + 2);
			*(pdata + 2) = (unsigned char)*(p + 1);
			*(pdata + 3) = (unsigned char)*p;//�Ͱ�λ
		}
		/**
		* @Data    2019-02-14 15:09
		* @brief   �����ֽ�����ת���ֽ�����,��λ��ǰ
		* @param   uint32_t �ĸ��ֽ�
		* @param	 uint8_t *���ֽ�ָ�� 
		* @retval  void
		*/
		void TwobyteToByle(int16_t data,uint8_t *pdata)
		{
			unsigned char *p;
			p = (unsigned char *)&data;
			*pdata = (unsigned char)*(p + 1);//�߰�λ
			*(pdata+1)= (unsigned char)*p;//�Ͱ�λ
		}
	/**
		* @Data    2019-02-14 15:45
		* @brief   ���ֽ�����ת�ĸ��ֽڣ���λ��ǰ
		* @param	 uint8_t *���ֽ�ָ�� 
		* @param   uint32_t *�ĸ��ֽ�ָ��
		* @retval  void
		*/
		void ByleToMultibyte(uint8_t *pdata,uint32_t *data)
		{
			*data = (uint32_t)((*pdata << 24) | (*(pdata + 1) << 16) | \
							(*(pdata + 2) << 8) |(*(pdata + 3)));
		}
/*--------------------------------file of end-------------------------------*/
