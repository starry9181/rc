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
	|--FileName    : dog_control.h                                                
	|--Version     : v1.0                                                            
	|--Author      : ����ƽͷ��                                                       
	|--Date        : 2019-09-10               
	|--Libsupports : 
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __DOG_CONTROL_H 
#define __DOG_CONTROL_H
#include "counters.h" 
#include "DJI_dbus.h"  
/*------------------------------------  �궨��  -------------------------------*/
#define D_RUNING   0 //����
#define D_STARING  1//��ʼ״̬
#define D_GAIT_OK  2//һ����̬���
#define MAX_LEG_HIGH 30//���̧�ȸ߶� ��λmm
#define MIN_LEG_HIGH 0//��С̧�ȸ߶�  ��λmm
#define LEG_F_CTRL_PIONT_S  (1.0f/3.0f)//̧�����߿��Ƶ�ռ�� ��1/3�������
#define BEZIER_BUFF_DATE_T 100//���������߾���
#define B_L_LEG      0//ǰ����
#define B_R_LEG      1//ǰ����
#define A_L_LEG      2//������
#define A_R_LEG      3//������
#define B_L_AND_A_R  1//ǰ��ͺ���
#define B_R_AND_A_L  0//ǰ�Ҹ�����
/*---------------------------------  �ṹ������  ------------------------------*/
typedef struct dogCtrlStruct
{
	int16_t high;//̧�ȸ߶�
	int16_t speed;//̧���ٶ�
  PointU32 in_t[3];//���Ƶ�
  PointU32 out_t[BEZIER_BUFF_DATE_T]; //�������滮����ռ�
} dogCtrlStruct;
typedef struct geometricForIK
{
  /*��׼����ǣ���ʱ��Ϊ��*/
  float q1;   //l1��ʼ�Ƕ�  ��λ ��
  float q2;   //l2��ʼ�Ƕ�  ��λ ��
  float l1;   //���ȳ���  ��λmm
  float l2;   //С�ȳ���  ��λmm
  float psi;  //��
  float last_q1; //q1�ϴνǶ�  ��λ �� ��ʼ��ʱһ��Ҫ  last_q1 = q1
  uint8_t status;//��ʼ��״̬ MOD_READΪδ��ʼ����INIT_OK��ʼ����
  /*��ʼ��ʾ��
   gIK_t.q1 = 45;   //l1��ʼ�Ƕ�  45��
   gIK_t.q2 = 45;   //l2��ʼ�Ƕ�  45��
   gIK_t.l1 = 100;   //���ȳ��� 100mm
   gIK_t.l2 = 100;   //С�ȳ��� 100mm
   gIK_t.psi = 0;  //��
   gIK_t.last_q1 = gIK_t.q1; //q1�ϴνǶ�  ��λ �� ��ʼ��ʱһ��Ҫ  last_q1 = q1
   gIK_t.status = INIT_OK;//��ʼ��״̬ MOD_READΪδ��ʼ����INIT_OK��ʼ��
  */
}geometricForIK;
/*-----------------------------------  ��������  ------------------------------*/
void DogControlInit(const dbusStruct* pRc_t);
void DogCtrl(void);
uint8_t TrajectoryPlanning(int16_t speed,int16_t high);
void CalBezierCtrlPoint(int16_t speed,int16_t high);
uint8_t FootControls(uint8_t w_leg);
SYS_Status GeometricForIK(geometricForIK *gIK_t,PointU32 point);
#endif
/*-----------------------------------file of end------------------------------*/


