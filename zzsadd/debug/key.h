#ifndef KEY_H
#define KEY_H
#include "config.h"
/*************** key funtion define************/
#define LONG_PRESS_TIME  		 1000//���������Ĵ���ʱ��
#define FORWARD_NORMAL    	 (RC_Ctl.key.bit.W)//ǰ��
#define FORWARD_VERY_SLOWLY  (RC_Ctl.key.bit.E)//����ǰ��
#define BACK        				 (RC_Ctl.key.bit.S)//����
#define LEFT       					 (RC_Ctl.key.bit.A)//����
#define RIGHT      					 (RC_Ctl.key.bit.D)//����
#define OPEN_MAGAZINE				 (RC_Ctl.key.bit.R)//�򿪵���
#define CLOSE_MAGAZINE       (RC_Ctl.key.bit.CTRL && RC_Ctl.key.bit.R)//�رյ���
#define	WANT_USE_CAP	   		 (RC_Ctl.key.bit.SHIFT)//ʹ�õ��ݷŵ�
#define MANUAL_ATTACK				 (RC_Ctl.mouse.r)//ȡ������
#define MANUAL_SHOOT				 (RC_Ctl.mouse.l_press)//����
#define ROBOT_ATTACK         (RC_Ctl.key.bit.Z)//�Ӿ�Ŀ��Ϊ������
#define SMALL_FAN_ATTACK		 (RC_Ctl.key.bit.X)//�Ӿ�Ŀ��ΪС��������
#define BIG_FAN_ATTACK       (RC_Ctl.key.bit.C)//�Ӿ�Ŀ��Ϊ����������
#define GIMBAL_LOCK_SW       (s_keymouse.F_state == KEY_PRESS_ONCE)//����/�˳�����̨ģʽ
#define CHASSIS__GYRO_SW     (s_keymouse.V_state == KEY_PRESS_ONCE)//����/�˳�С����ģʽ 

#endif
