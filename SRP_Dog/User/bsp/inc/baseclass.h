/**
	|-------------------------------- Copyright |--------------------------------|
	|                                                                            |
	|                        (C) Copyright 2018,����ƽͷ��,                       |
	|         1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China        |
	|                         All Rights Reserved                                |
	|                                                                            |
	|          By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)          |
	|                   https://github.com/GCUWildwolfteam                       |
	|----------------------------------------------------------------------------|
	|--FileName    : baseclass.h
	|--Version     : v1.0
	|--Author      : ����ƽͷ��
	|--Date        : 2018-11-27 
	|-- Libsupports:           
	|--Description :1.���ú궨�壬ö�٣�������
	|--FunctionList
	|-------1. ....
	|          <version>:
	|     <modify staff>:
	|             <data>:
	|      <description>:
	|-------2. ...
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __BASECLASS_H 
#define __BASECLASS_H 
#include "userdriverconfig.h"
/* -------------- ��������� ----------------- */
	#define MAX(X,Y) 							((X)>(Y)?(Y):(X))//������ֵ
	#define MIN(X,Y) 							((X)<(Y)?(Y):(X))//����Сֵ
	#define ABS(X)   							((X)<0?(-X):(X))//ȡ����ֵ
  #define ARR_SIZE(a)  					(sizeof((a))/sizeof((a[0])))//��������Ԫ�صĸ���
  #define INC_SAT(val) (val=((val)+1>(val)) ? (val)+1 : (val))//��ֹ�����һ������
  #define HEXCHK(c) (((c) >= '0' && (c)<='9') ((c)>='A' && (c)<= 'F') \
                     ((c)>='a' && (c)<='f'))//�ж��ַ��ǲ���16��ֵ����
  #define  DECCHK(c) ((c)>='0' && (c)<='9')//�ж��ַ��ǲ���10��ֵ������
  #define  RND8(x)						((((x) + 7)/8) * 8)//����һ����X�����ӽ���8�ı���
  #define  WORD_LO(xxx)       ((byte) ((word)(xxx) & 255))//�õ�һ���ֵ�λ�ֽ�
  #define  WORD_HI(xxx)         ((byte) ((word)(xxx) >> 8))//��һ���ֵĸ�λ��
	/*�õ�һ���ṹ����field��ռ�õ��ֽ���*/		
  #define  FSIZ(type,field)     sizeof(((type *)0)->field)
	/*�õ�һ��field�ڽṹ��(struct)�е�ƫ����*/	
  #define  FPOS(type,field)     ((dword)&((type *)0)->field)
  #define  MEM_B(x)             (*((byte *)(x)))// �õ�ָ����ַ�ϵ�һ���ֽ�
  #define  MEM_W(x)             (*((word *)(x)))//�õ�ָ����ַ�ϵ�һ����
  #define  IS_CLOSE_INT(x,a,b)  (x>=a && x<=b)//�ж��Ƿ���һ����������
  #define  IS_OPEN_INT(x,a,b)   (x>a && x<b)//�ж��Ƿ���һ����������
  #define  LEFT_SH(_a,_x)            (_a <<_x)//����_xλ
  #define  RIGHT_SH(_a,_x)           (_a >>_x)//����_xλ
  #define  GET_BYTE(_a,_x)           (_a & _x)//��ȡ_xλ
  #define  IS_BYTE(_a,_x)            ((_a & _x) ==_x)//�ж϶�Ӧ��λ�Ƿ����
	#define	CYCLE_NUMERICAL(data,max) ((data+1)%max)//����ѭ��
  #define PI (3.14159f) //��
/* ----------------- ����״̬λ �� -------------------- */
/* uint32_t status(0������͵�λ��31�������λ)��Ϊ�˷������������־λ��
 * -------------------------------------------------
 *������ |   0bit   |   1bit    |   2bit   |    3bit    | 7~3bit |
 *�ݳ�ʼ--------------------------------------------
 *��λ   | ��ʼ���ɹ�| �������� | �������� |�������óɹ�| ����  |
 * ----------------------------------------------------------------
 * ����| 8bit | 9bit | 10bit  | 11bit  | 12bit  | 13bit | 14~15bit | 
 * ��ʼ-------------------------------------------------------------
 * ��λ| can1 | CAN2 | UASRT1 | UASRT3 | UASRT6 | UART7 |   ����    | 
 * -----------------------------------------------------------------
 * |        24~31bit      | 
 * -------------------------
 * |  ����(ģ���Զ���λ)  | 
 * -------------------------
 */
	#define MOD_READ                    0x00000000U//ģ��׼������
	#define INIT_OK                     0x00000001U//��ʼ���ɹ�
  #define RUNING_OK                   0x00000002U//��������
  #define RX_OK                       0x00000004U//��������
  #define START_OK                    0x00000008U//�������óɹ�
  #define DHECK_DATA_RUNING           0x00000010U//����У׼��������   
/* -------------- ����״̬���� ----------------- */
  #define IS_INIT_OK(_status_)      IS_BYTE(_status_,INIT_OK)
  #define IS_RUNING_OK(_status_)    IS_BYTE(_status_,RUNING_OK)
  #define IS_RX_OK(_status_)        IS_BYTE(_status_,RX_OK)
  #define IS_START_OK(_status_)     IS_BYTE(_status_,START_OK)
	/* --��������һ��Ҫһ���ã�����֮��һ��Ҫ�ͷ� -- */
	#define CACHE_ADDR(CACHES,ADDRS) 	(CACHES = ADDRS) //�����ַ
	#define FREE_ADDR(CACHES) 	      (CACHES = NULL)		//�ͷŵ�ַ  
/* -------------- ����ת�������� ----------------- */
typedef union  
{   
  float  f;   
  int16_t s_16[2];
  uint16_t u_16[2];
  unsigned char u_8[4];
   uint32_t u_32; 
   int32_t s_32;  
}floatToUnion; 
/* -------------- ϵͳö�� ----------------- */
	typedef enum
	{
		SYS_OK       = 0x00U,
		SYS_ERROR    = 0x01U,
		SYS_BUSY     = 0x02U,
		SYS_LOST     = 0x03U
	}SYS_Status;
/* -------------- ģ��״̬ö�� ----------------- */
typedef enum 
{
  MOD_OK       = 0x00U,
  MOD_ERROR    = 0x01U,
  MOD_BUSY     = 0x02U,
  MOD_LOST     = 0x03U
}MOD_Status;
/* -------------- �������� ----------------- */
	void* Get_Peripheral_type(void* Peripheral);
	void MultibyteToByle(uint32_t data,uint8_t *pdata);
	void ByleToMultibyte(uint8_t *pdata,uint32_t *data);
	void TwobyteToByle(int16_t data,uint8_t *pdata);
#endif	// __BASECLASS_H
	
 /*--------------------------------file of end--------------------------------*/

