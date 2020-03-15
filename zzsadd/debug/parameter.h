#ifndef PARAMETER_H
#define PARAMETER_H
#include "config.h"
/************ constant *******************************/
#define ANGLE_ENCODE            	22.7527778f//�ǶȻ��������ֵ��8191/360��
#define ENCODE_ANGLE            	0.0439506f //����ֵ����ɽǶȣ�360/8191��
#define RPM_DPS                 	6 //תÿ�ֻ���ɶ�ÿ�루1/60*360��
#define ANGLE_RAD               	0.0174532f //�ǶȻ���ɻ��ȣ�3.1415926/180��
#define RAD_ANGLE               	57.29578f//���Ȼ���ɽǶ�
/************ chassis *******************************/
#define MOVE_WSCONST  						8000//����ǰ���ٶ�����
#define MOVE_ADCONST  						4000//���������ٶ�����
#define E_WSCONST 	  						3000//����ǰ��ʱ�ٶ�����
#define CHANNEL_CONST             12.12f//�����ٶ�ң����ͨ��������660*12.12=8000��
#define CHASSIS_MAX_SPEED         8000//���̵�������ٶ�
#define SCALE_V_W                 0.35f//С����ģʽ��ƽ���ٶ�ռ��ת�ٶȵ����ռ��
/************ gimbal *************************************/
#define CHANNEL_YAW_CONST         -0.048f//yawң����ͨ������
#define CHANNEL_PITCH_CONST       -0.012f//pitchң����ͨ������
#define MOUSE_YAW_CONST           -0.8f//yaw��곣��
#define MOUSE_PITCH_CONST         -0.2f//pitch��곣��
#define TRANS_COMP_CONST 					0.09f//����yaw������Ĳ��������ϵ��
/************ shoot ***************************************/
#define TRAVEL                    29487.6f//���̵ĵ����г̣�8191*36/10��
/************ vision ************************************/
#if  ROBOT_ID == 1
#define FRAMEDIFF_TO_SPD    			3.0f//֡��ת�����ٶȵ�ϵ��
#define FANWHEEL_CENTER_X   			250.0f//������������x����
#define FANWHEEL_CENTER_Y					242.0f//������������y����
#define ARMOR_OFFSET_X      			800.0f//���������x����
#define ARMOR_OFFSET_Y      			280.0f//���������y����
#define FAN_COMP_COEF       			0.01f//��������pitch����ϵ��
#define ROBOT_PIT_COEF      			1.0f//װ�װ�pitch����ϵ��
#define ROBOT_YAW_COEF      			0.4f//ʶ��װ��ʱ�ٶȲ���ϵ��
#define FAN_SHOOT_ERR       			1.0f//����������С�������
#define ROBOT_SHOOT_YAW_ERR 			2.0f//������x������С�������
#define ROBOT_SHOOT_PIT_ERR 			1.5f//������y������С�������
#define ROBOT_SHOOT_SPD_ERR 			150.0f//�������ٶ���С�������
#elif ROBOT_ID == 2

#elif ROBOT_ID == 3

#endif
#endif
