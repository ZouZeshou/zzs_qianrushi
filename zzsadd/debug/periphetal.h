#ifndef PERIPHETAL_H
#define PERIPHETAL_H
#include "config.h"
/**************************** uart ********************************************/
/*attention:���Ĵ���֮�����ڡ�stm32f4xx_it.c�������жϺ���*/
#define DBUS_UART             USART1//ң��������
#define DBUS_HANDLE           huart1
#define DBUS_DMA              hdma_usart1_rx
#define DBUS_LENGTH           18
#define DEBUG_URAT            USART2//���Դ���
#define DEBUG_HANDLE          huart2
#define DEBUG_LENGTH          1
#define VISION_UART           UART7//�Ӿ�����
#define VISION_HANDLE         huart7
#define VISION_DMA            hdma_uart7_rx
#define VISION_LENGTH         23
/**************************** can receive id**********************************/
/* can2 */
#define CHASSIS_M1_ID					0x201//LF motor������ǰ�����
#define CHASSIS_M2_ID					0x202//RF motor������ǰ�����
#define CHASSIS_M3_ID					0x203//LB motor������󷽵��
#define CHASSIS_M4_ID					0x204//RB motor�����Һ󷽵��
#define YAW_ID								0x205//yaw�������� 
#define	GIMBALIMU_ID_1				0x401//����pitch�ϵ�����������
#define CHASSISIMU_ID					0x402//���ڵ��̵�����������
#define JudgeGameState_ID			0x101//����ϵͳ���ݶ�Ӧid
#define JudgeRobotState_ID		0x102
#define JudgePowerHeat1_ID		0x103
#define JudgePowerHeat2_ID		0x104
#define JudgeRobotPos1_ID			0x105
#define JudgeRobotPos2_ID			0x106
#define JudgeBuffMusk_ID			0x107
#define JudgeRobotHurt_ID			0x111
#define JudgeShootData_ID			0x112
#define SuperCapInfo_ID				0x308//������������id
/* can1 */
#define L_FRI_WHEEL_ID				0x201//���Ħ����
#define R_FRI_WHEEL_ID				0x202//�ұ�Ħ����
#define TRANS_ID						  0x203//���̵��
#define PIT_ID								0x206//pitch��������
#endif
