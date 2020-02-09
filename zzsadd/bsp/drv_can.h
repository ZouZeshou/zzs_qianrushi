#ifndef DRV_CAN__H
#define DRV_CAN__H
#include "global.h"
#include "can.h"
void Can_SendMsg_by_byte(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t data[8]);
void Can_SendMsg(CAN_HandleTypeDef *hcan,uint32_t id,int16_t data1,int16_t data2,int16_t data3,int16_t data4);
void CANFilter_Enable(CAN_HandleTypeDef *hcan);
void CAN_Enable(CAN_HandleTypeDef *hcan);
/**************************** can receive id**********************************/
/* can2 */
#define CHASSIS_M1_ID					0x201		//LF motor
#define CHASSIS_M2_ID					0x202		//RF motor
#define CHASSIS_M3_ID					0x203		//LD motor
#define CHASSIS_M4_ID					0x204		//RD motor
#define YAW_ID								0x205
#define	GIMBALIMU_ID_1				0x401
#define CHASSISIMU_ID					0x402
#define JudgeGameState_ID			0x101
#define JudgeRobotState_ID		0x102
#define JudgePowerHeat1_ID		0x103
#define JudgePowerHeat2_ID		0x104
#define JudgeRobotPos1_ID			0x105
#define JudgeRobotPos2_ID			0x106
#define JudgeBuffMusk_ID			0x107
#define JudgeRobotHurt_ID			0x111
#define JudgeShootData_ID			0x112
#define SuperCapInfo_ID				0x308
/* can1 */
#define GIMBAL_CENTER_ID1 		0x101
#define GIMBAL_CENTER_ID2			0x102
#define L_FRI_WHEEL_ID				0x201
#define R_FRI_WHEEL_ID				0x202
#define TRANS_ID						  0x203
#define PIT_ID								0x206
#endif
