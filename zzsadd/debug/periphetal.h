#ifndef PERIPHETAL_H
#define PERIPHETAL_H
#include "config.h"
/**************************** uart ********************************************/
/*attention:更改串口之后还需在“stm32f4xx_it.c”更改中断函数*/
#define DBUS_UART             USART1//遥控器串口
#define DBUS_HANDLE           huart1
#define DBUS_DMA              hdma_usart1_rx
#define DBUS_LENGTH           18
#define DEBUG_URAT            USART2//调试串口
#define DEBUG_HANDLE          huart2
#define DEBUG_LENGTH          1
#define VISION_UART           UART7//视觉串口
#define VISION_HANDLE         huart7
#define VISION_DMA            hdma_uart7_rx
#define VISION_LENGTH         23
/**************************** can receive id**********************************/
/* can2 */
#define CHASSIS_M1_ID					0x201//LF motor底盘左前方电机
#define CHASSIS_M2_ID					0x202//RF motor底盘右前方电机
#define CHASSIS_M3_ID					0x203//LB motor底盘左后方电机
#define CHASSIS_M4_ID					0x204//RB motor底盘右后方电机
#define YAW_ID								0x205//yaw航向轴电机 
#define	GIMBALIMU_ID_1				0x401//放在pitch上的自制陀螺仪
#define CHASSISIMU_ID					0x402//放在底盘的自制陀螺仪
#define JudgeGameState_ID			0x101//裁判系统数据对应id
#define JudgeRobotState_ID		0x102
#define JudgePowerHeat1_ID		0x103
#define JudgePowerHeat2_ID		0x104
#define JudgeRobotPos1_ID			0x105
#define JudgeRobotPos2_ID			0x106
#define JudgeBuffMusk_ID			0x107
#define JudgeRobotHurt_ID			0x111
#define JudgeShootData_ID			0x112
#define SuperCapInfo_ID				0x308//超级电容数据id
/* can1 */
#define L_FRI_WHEEL_ID				0x201//左边摩擦轮
#define R_FRI_WHEEL_ID				0x202//右边摩擦轮
#define TRANS_ID						  0x203//拨盘电机
#define PIT_ID								0x206//pitch俯仰轴电机
#endif
