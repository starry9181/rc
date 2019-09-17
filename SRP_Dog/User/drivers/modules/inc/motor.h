/**
	|------------------------------- Copyright ----------------------------------|
	|                                                                            |
	|                      (C) Copyright 2018,����ƽͷ��,                         |
	|          1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                         All Rights Reserved                            -   |
	|                                                                            |
	|          By(GCU The wold of team | ��������ѧ����ѧԺ������Ұ�Ƕ�)       -  |
	|                     https://github.com/GCUWildwolfteam                     |
	|----------------------------------------------------------------------------|
	|--FileName    : motor.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2018-12-31               
	|--Libsupports : ��׼���HAL��
	|--Description : 1��maxion���+RoboModule���� 
	|								 2��3508���+c610���
	|								 3��6623���
	|                4��6002���                                  
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|------------------------------declaration of end----------------------------|
 **/
#ifndef __MOTOR_H 
#define __MOTOR_H
	#include "baseclass.h "
	#include "bsp_can.h" 
	#include "counters.h" 
/* =========================== common of begin =========================== */
	int16_t RatiometricConversion(int16_t real,int16_t threshold,int16_t perce,int32_t* last_real,int16_t* coefficient,uint32_t status);
  	int16_t NoRatiometricConversion(int16_t real,int16_t threshold,int16_t perce,int32_t* last_real,int16_t* coefficient,uint32_t status);
	int16_t zeroArgument(int16_t real, int16_t threshold);
/* =========================== common of end =========================== */
/* =========================== maxion of begin =========================== */
 /* -------- ID�� --------- */
	// typedef enum
	// {
	// 	openloop_mode_id_e                       = 0x01,//����ģʽid��
	// 	current_mode_id_e                        = 0x02,//����ģʽid��
	// 	velocity_mode_id_e                       = 0x03,//�ٶ�ģʽid��
	// 	position_mode_id_e                       = 0x04,//λ��ģʽid��
	// 	velocity_position_mode_id_e              = 0x05,//�ٶ�λ��ģʽid��
	// 	current_velocity_mode_id_e               = 0x06,//�����ٶ�ģʽid��
	// 	current_position_mode_id_e               = 0x07,//����λ��ģʽid��
	// 	current_velocity_position_mode_id_e      = 0x08 //����λ���ٶ�ģʽid�� 
	// }motorIDEnum;

		#define openloop_mode_id_e                       0x01//����ģʽid��
		#define current_mode_id_e                        0x02//����ģʽid��
		#define velocity_mode_id_e                       0x03//�ٶ�ģʽid��
		#define position_mode_id_e                       0x04//λ��ģʽid��
		#define velocity_position_mode_id_e              0x05//�ٶ�λ��ģʽid��
		#define current_velocity_mode_id_e               0x06//�����ٶ�ģʽid��
		#define current_position_mode_id_e               0x07//����λ��ģʽid��
		#define current_velocity_position_mode_id_e      0x08//����λ���ٶ�ģʽid�� 

		/* -------- ����ģʽ --------- */
	// typedef enum
	// {
	// 	openloop_mode_e                       = 0x01,//����ģʽ
	// 	current_mode_e                        = 0x02,//����ģʽ
	// 	velocity_mode_e                       = 0x03,//�ٶ�ģʽ
	// 	position_mode_e                       = 0x04,//λ��ģʽ
	// 	velocity_position_mode_e              = 0x05,//�ٶ�λ��ģʽ
	// 	current_velocity_mode_e               = 0x06,//�����ٶ�ģʽ
	// 	current_position_mode_e               = 0x07,//����λ��ģʽ
	// 	current_velocity_position_mode_e      = 0x08//����λ���ٶ�ģʽ
	// }motorModeEnum;

	#define	openloop_mode_e                       0x01//����ģʽ
	#define	current_mode_e                        0x02//����ģʽ
	#define	velocity_mode_e                       0x03//�ٶ�ģʽ
	#define	position_mode_e                       0x04//λ��ģʽ
	#define	velocity_position_mode_e              0x05//�ٶ�λ��ģʽ
	#define	current_velocity_mode_e               0x06//�����ٶ�ģʽ
	#define	current_position_mode_e               0x07//����λ��ģʽ
	#define	current_velocity_position_mode_e      0x08//����λ���ٶ�ģʽ
	/* ------ ������� ------- */
		// typedef enum
		// {
		// 	reset_fun_e                          = 0x00,//��λָ��
		// 	mode_selection_fun_e                 = 0x01,//ģʽѡ��
		// 	openloop_fun_e                       = 0x02,//����ģʽ
		// 	current_fun_e                        = 0x03,//����ģʽ
		// 	velocity_fun_e                       = 0x04,//�ٶ�ģʽ
		// 	position_fun_e                       = 0x05,//λ��ģʽ
		// 	velocity_position_fun_e              = 0x06,//�ٶ�λ��ģʽ
		// 	current_velocity_fun_e               = 0x07,//�����ٶ�ģʽ
		// 	current_position_fun_e               = 0x08,//����λ��ģʽ
		// 	current_velocity_position_fun_e      = 0x09,//����λ���ٶ�ģʽ
		// 	config_fun_e 												 = 0x0A,//����ָ��
		// 	data_feedback_fun_e 								 = 0x0B,//���ݷ���
		// 	left_and_right_limit_feedback_e 		 = 0x0C,//������λ����
		// 	online_check_fun_e 		 							 = 0x0F//���߼��
		// }functionNumEnum;

		#define	reset_fun_e                          0x00//��λָ��
		#define	mode_selection_fun_e                 0x01//ģʽѡ��
		#define	openloop_fun_e                       0x02//����ģʽ
		#define	current_fun_e                        0x03//����ģʽ
		#define	velocity_fun_e                       0x04//�ٶ�ģʽ
		#define	position_fun_e                       0x05//λ��ģʽ
		#define	velocity_position_fun_e              0x06//�ٶ�λ��ģʽ
		#define	current_velocity_fun_e               0x07//�����ٶ�ģʽ
		#define	current_position_fun_e               0x08//����λ��ģʽ
		#define	current_velocity_position_fun_e      0x09//����λ���ٶ�ģʽ
		#define	config_fun_e 												 0x0A//����ָ��
		#define	data_feedback_fun_e 								 0x0B//���ݷ���
		#define	left_and_right_limit_feedback_e 		 0x0C//������λ����
		#define	online_check_fun_e 		 							 0x0F//���߼��
	typedef struct maxionStruct
	{
		int16_t real_current; //��ʵ����
		int16_t real_position;//��ʵ�Ƕ�
		int16_t real_velocity;//��ʵ�ٶ�
		uint32_t module_id;
    uint32_t module_rx_id;//������ݽ���id
		CAN_HandleTypeDef* hcanx;
	}maxionStruct;
	/* -------------- ���к� ----------------- */
	#define MOTOR_CAN_ID_CAL(group,number) \
						((((group)<<8)|((number)<<4)))	//���can ����id����
	void ResetMode(CAN_HandleTypeDef *hcanx,uint32_t can_rx_id);
	void ModeSelectionMode(CAN_HandleTypeDef *hcanx,uint32_t rx_id,uint8_t mode);
	void OpenLoopMode(int16_t pwm,uint8_t *data);
  void SpeedLoopMode(int16_t pwm,int16_t Speed,uint8_t *data);
	void ConfigMode(CAN_HandleTypeDef *hcanx,uint32_t rx_id,uint8_t Time,\
																												uint8_t Ctl1_Ctl2);
	void MotorInit(CAN_HandleTypeDef *hcanx,uint8_t id,uint8_t mode);
  void MaxionParseData(maxionStruct*maxion,uint8_t *data);
  int16_t CalculateError(int16_t target,int16_t real,int16_t max_speed,int16_t linesnumb);
  int16_t CalculatePationError(int16_t target,int16_t real);
/* =========================== maxion of end =========================== */

/* ============================= RM6623 of begin ============================ */
  #define RM6623_LIM    13000//6623������ֵ -13000~13000
	/* -------------- �ṹ�� ----------------- */	
	typedef struct RM6623Struct
	{
		uint16_t id;//���can�� ip
		int16_t target;		 //Ŀ��ֵ
		int16_t tem_target;//��ʱĿ��ֵ
		int16_t real_current; //��ʵ����
		int16_t real_angle;//��ʵ�Ƕ�
		int16_t tem_angle;//��ʱ�Ƕ�
		int16_t zero;			 //������
		int16_t Percentage;//ת������������ǰ�Ƕ�:���ٺ�ĽǶ� = x:1
		int16_t thresholds; //�����ת��ֵ
    int16_t error;//��ǰ���
    int32_t last_real;
    int16_t coefficient;
    fps_t fps;//֡��
    postionPidStruct *ppostionPid_t;
    postionPidStruct* InnerLoopPid_t;
		//speedPidStruct *pspeedPid_t;
    
	} RM6623Struct;
	/* -------------- ���к��� ----------------- */
	void RM6623StructInit(RM6623Struct *RM6623,CAN_HandleTypeDef *hcanx);
	void RM6623ParseData(RM6623Struct*RM6623,uint8_t *data);
/* ============================= RM6623 of end ============================== */
/* ============================ RM3508 of begin ============================= */
  #define RM3508_LIM    16384//RM3508������ֵ -16384~16384
	typedef struct RM3508Struct
	{
		uint16_t id;//���can�� ip
		int16_t target;		 //Ŀ��ֵ
		int16_t tem_target;//��ʱĿ��ֵ
		int16_t real_current; //��ʵ����
		int16_t real_angle;//��ʵ�Ƕ�
		int16_t tem_angle;//��ʱ�Ƕ�
    int16_t real_speed;//��ʵ�ٶ�
    int16_t tem_speed;//��ʵ�ٶ�
		int16_t zero;			 //������
		int16_t Percentage;//ת������������ǰ�Ƕ�:���ٺ�ĽǶ� = x:1
		int16_t thresholds; //�����ת��ֵ
    int16_t error;//��ǰ���
    fps_t fps;//֡��
    postionPidStruct *ppostionPid_t;
		speedPidStruct *pspeedPid_t;
	}RM3508Struct;
void RM3508ParseData(RM3508Struct *RM3508,uint8_t *data);
/* ============================== Rm3508 of end ============================= */
/* =========================== M2006 of begin =========================== */
  #define RM2006_LIM    10000//M2006������ֵ -10000~10000
	typedef struct M2006Struct
	{
		uint16_t id;//���can�� ip
		int16_t target;		 //Ŀ��ֵ
		int16_t tem_target;//��ʱĿ��ֵ
		int16_t real_current; //��ʵ����
		int16_t real_angle;//��ʵ�Ƕ�
		int16_t tem_angle;//��ʱ�Ƕ�
		int16_t zero;			 //������
		int16_t Percentage;//ת������������ǰ�Ƕ�:���ٺ�ĽǶ� = x:1
		int16_t thresholds; //�����ת��ֵ
    int16_t error;//��ǰ���
		int16_t real_speed;//��ʵ�ٶ�
    int16_t tem_speed;//��ʵ�ٶ�
    int32_t last_real;
    int16_t coefficient;
    fps_t fps;//֡��
    postionPidStruct *ppostionPid_t;
		speedPidStruct *pspeedPid_t;
	}M2006Struct;
	void RM2006ParseData(M2006Struct *M2006, uint8_t *data);
  void AntiRM2006ParseData(M2006Struct *M2006,uint8_t *data);
	#define M2006_THRESHOLD 6000
	#define M2006_POLES  8192
/* =========================== M2006 of end =========================== */
/* =========================== PWM���Ƶĵ�� of begin ======================== */
	void BrushlessMotorInit(void);
	 #define SET_FRICTIONGEAR_SPEED(__V_)															 \
				do{                                                  				\
						__HAL_TIM_SetCompare(FRICTIONGEAR,FRICTIONGEAR_1,__V_);  \
						__HAL_TIM_SetCompare(FRICTIONGEAR,FRICTIONGEAR_2,__V_);  \
					} while(0U)//����Ħ�����ٶ�
/* =========================== PWM���Ƶĵ�� of end ========================== */
/* =========================== M6020 of begin =========================== */
  #define RM6020_LIM    30000//M6020������ֵ -30000~30000
  #define RM6020_LINES  8192//�������
	typedef struct GM6020Struct
{
		uint16_t id;//���can�� ip
		int16_t target;		 //Ŀ��ֵ
		int16_t tem_target;//��ʱĿ��ֵ
		int16_t real_current; //��ʵ����
		int16_t real_angle;//��ʵ�Ƕ�
		int16_t tem_angle;//��ʱ�Ƕ�
		int16_t zero;			 //������
		int16_t Percentage;//ת������������ǰ�Ƕ�:���ٺ�ĽǶ� = x:1
		int16_t thresholds; //�����ת��ֵ
    int16_t error;//��ǰ���
		int16_t real_speed;//��ʵ�ٶ�
    int16_t tem_speed;//��ʵ�ٶ�
    int32_t last_real;
    int16_t coefficient;
    fps_t fps;//֡��
    postionPidStruct *ppostionPid_t;
//		speedPidStruct *pspeedPid_t;
  postionPidStruct *pspeedPid_t;
}GM6020Struct;
 void GM6020ParseData(GM6020Struct* GM6020,uint8_t *data);
/* =========================== M6020 of end =========================== */


#endif	// __MOTOR_H
/*---------------------------------file of end--------------------------------*/


