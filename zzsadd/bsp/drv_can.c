#include "drv_can.h"
#include "shoot.h"
#include "chassis.h"
#include "gimbal.h"
#include "drv_judgesystem.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "monitor_task.h"
/**
 * @brief Enable Can1 and Can2(对can1和can2进行初始化)
 * @param None
 * @return None
 * @attention None
 */
void CAN_Enable(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);//对can进行激活
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//使能can接收中断
	CANFilter_Enable(hcan);//使能滤波器
}

/**
 * @brief interrupt function in IRQ（can接收中断回调函数）
 * @param None
 * @return None
 * @attention None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	wl2data u_2data;
	wl4data u_4data;
	static uint8_t RxData1[8],RxData2[8];//定义用于接收消息的数组
	CAN_RxHeaderTypeDef Can1Header,Can2Header;//定义接收函数需要用到的句柄
	//如果是can1
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&Can1Header,RxData1 );//从FIFO邮箱中读取消息至RxData1
		//判断can总线ID，对应电机
		switch(Can1Header.StdId)
		{	
			case L_FRI_WHEEL_ID:
			{
				s_fric_l_motor.back_position = RxData1[0]<<8|RxData1[1];
				s_fric_l_motor.back_speed = RxData1[2]<<8|RxData1[3];
				s_fps.fric_l++;
				break;
			}
			case R_FRI_WHEEL_ID:
			{
				s_fric_r_motor.back_position = RxData1[0]<<8|RxData1[1];
				s_fric_r_motor.back_speed = RxData1[2]<<8|RxData1[3];
				s_fps.fric_r++;
				break;
			}
			case TRANS_ID:
			{
				s_trans_motor.back_position = RxData1[0]<<8|RxData1[1];
				s_trans_motor.back_speed = RxData1[2]<<8|RxData1[3];
				continue_motor_pos(&s_trans_motor);
				s_fps.trans++;
				break;
			}
			case PIT_ID:
			{
				s_pitch_motor.back_position = RxData1[0]<<8|RxData1[1];
				s_pitch_motor.back_speed = RxData1[2]<<8|RxData1[3];
				s_pitch_motor.temperature = RxData1[6];
				continue_motor_pos(&s_pitch_motor);
				s_fps.pitch++;
				break;
			}
			default:
				break;
		}
	}
	//如果是can2
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0,&Can2Header,RxData2 );//从FIFO中接收消息至RxData2
		switch(Can2Header.StdId)
		{
			case CHASSIS_M1_ID:
			{
				s_chassis_motor[0].back_position = RxData2[0]<<8|RxData2[1];
				s_chassis_motor[0].back_speed = RxData2[2]<<8|RxData2[3];
				s_chassis_motor[0].back_current = RxData2[4]<<8|RxData2[5];
				s_chassis_motor[0].temperature = RxData2[6];
				s_fps.chassis[0]++;
				break;
			}
			case CHASSIS_M2_ID:
			{
				s_chassis_motor[1].back_position = RxData2[0]<<8|RxData2[1];
				s_chassis_motor[1].back_speed = RxData2[2]<<8|RxData2[3];
				s_chassis_motor[1].temperature = RxData2[6];
				s_fps.chassis[1]++;
				break;
			}
			case CHASSIS_M3_ID:
			{
				s_chassis_motor[2].back_position = RxData2[0]<<8|RxData2[1];
				s_chassis_motor[2].back_speed = RxData2[2]<<8|RxData2[3];
				s_chassis_motor[2].temperature = RxData2[6];
				s_fps.chassis[2]++;
				break;
			}
			case CHASSIS_M4_ID:
			{
				s_chassis_motor[3].back_position = RxData2[0]<<8|RxData2[1];
				s_chassis_motor[3].back_speed = RxData2[2]<<8|RxData2[3];
				s_chassis_motor[3].temperature = RxData2[6];
				s_fps.chassis[3]++;
				break;
			}
			case YAW_ID:
			{
				s_yaw_motor.back_position = RxData2[0]<<8|RxData2[1];
				s_yaw_motor.back_speed = RxData2[2]<<8|RxData2[3];
				s_yaw_motor.temperature = RxData2[6];
				continue_motor_pos(&s_yaw_motor);
				s_fps.yaw++;
				break;
			}
			case GIMBALIMU_ID_1:
			{
				wl2data pitch_gyro_spd;
				pitch_gyro_spd.c[0] = RxData2[0];
				pitch_gyro_spd.c[1] = RxData2[1];
				s_pitch_motor.gyro_speed = pitch_gyro_spd.d/16.384f;//converge speed to dps
				s_fps.pitch_imu++;
				break;
			}
			case CHASSISIMU_ID:
			{
				wl2data chas_gyro_spd;
				wl4data chas_gyro_ang;
				chas_gyro_spd.c[0] = RxData2[0];
				chas_gyro_spd.c[1] = RxData2[1];
				s_chassis.gyro_spd = -chas_gyro_spd.d/16.384f;//converge speed to dps
				chas_gyro_ang.c[0] = RxData2[4];
				chas_gyro_ang.c[1] = RxData2[5];
				chas_gyro_ang.c[2] = RxData2[6];
				chas_gyro_ang.c[3] = RxData2[7];
				s_chassis.gyro_angle = chas_gyro_ang.f;
				s_fps.chas_imu++;
				break;
			}
			case JudgeGameState_ID:
			{
				Judge_GameState.game_progress = RxData2[0];
				break;
			}
			case JudgeRobotState_ID:
			{
				Judge_GameRobotState.robot_level = RxData2[0];
				u_2data.c[0] = RxData2[1];
				u_2data.c[1] = RxData2[2];
				Judge_GameRobotState.remain_HP = u_2data.ud;
				Judge_GameRobotState.robot_id = RxData2[3];
				break;
			}
			case JudgePowerHeat1_ID:
			{
				u_2data.c[0] = RxData2[0];
				u_2data.c[1] = RxData2[1];
				Judge_PowerHeatData.chassis_volt = u_2data.ud;
				
				u_2data.c[0] = RxData2[2];
				u_2data.c[1] = RxData2[3];
				Judge_PowerHeatData.chassis_current = u_2data.ud;

				u_4data.c[0] = RxData2[4];
				u_4data.c[1] = RxData2[5];
				u_4data.c[2] = RxData2[6];
				u_4data.c[3] = RxData2[7];
				Judge_PowerHeatData.chassis_power = u_4data.f;
				s_fps.judge++;
				break;
			}
			case JudgePowerHeat2_ID:
			{
				/*JudgeFresh = xTaskGetTickCountFromISR();do not know the function*/
				u_2data.c[0] = RxData2[0];
				u_2data.c[1] = RxData2[1];
				Judge_PowerHeatData.chassis_power_buffer = u_2data.ud;
				
				u_2data.c[0] = RxData2[2];
				u_2data.c[1] = RxData2[3];
				Judge_PowerHeatData.shooter_heat0 = u_2data.ud;

				u_2data.c[0] = RxData2[4];
				u_2data.c[1] = RxData2[5];
				Judge_PowerHeatData.shooter_heat1 = u_2data.ud;
				break;
			}
			case JudgeRobotPos1_ID:
			{
				u_4data.c[0] = RxData2[0];
				u_4data.c[1] = RxData2[1];
				u_4data.c[2] = RxData2[2];
				u_4data.c[3] = RxData2[3];
				Judge_GameRobotPos.x = u_4data.f;

				u_4data.c[0] = RxData2[4];
				u_4data.c[1] = RxData2[5];
				u_4data.c[2] = RxData2[6];
				u_4data.c[3] = RxData2[7];
				Judge_GameRobotPos.y = u_4data.f;
				break;
			}
			case JudgeRobotPos2_ID:
			{
				u_4data.c[0] = RxData2[0];
				u_4data.c[1] = RxData2[1];
				u_4data.c[2] = RxData2[2];
				u_4data.c[3] = RxData2[3];
				Judge_GameRobotPos.z = u_4data.f;

				u_4data.c[0] = RxData2[4];
				u_4data.c[1] = RxData2[5];
				u_4data.c[2] = RxData2[6];
				u_4data.c[3] = RxData2[7];
				Judge_GameRobotPos.yaw = u_4data.f;
				break;
			}
			case JudgeBuffMusk_ID:
			{
				Judge_BuffMusk.buff.can_data = RxData2[0];
				break;
			}
			case JudgeRobotHurt_ID:
			{
				Judge_RobotHurt.armor_id =  RxData2[0];
				Judge_RobotHurt.hurt_type =  RxData2[1];
				break;
			}
			case JudgeShootData_ID:
			{
				Judge_ShootData.bullet_type = RxData2[0];
				Judge_ShootData.bullet_freq = RxData2[1];

				u_4data.c[0] = RxData2[2];
				u_4data.c[1] = RxData2[3];
				u_4data.c[2] = RxData2[4];
				u_4data.c[3] = RxData2[5];
				Judge_ShootData.bullet_speed = u_4data.f;
				break;
			}
			case SuperCapInfo_ID:
			{
				u_4data.c[0] = RxData2[0];
				u_4data.c[1] = RxData2[1];
				u_4data.c[2] = RxData2[2];
				u_4data.c[3] = RxData2[3];
				/*chassis.Cap_Volt = u_4data.f; 
				chassis.can_use_cap = RxData2[4];*///no define now
				break;
			}
			default:
				break;
		}
	
	}
}
/**
 * @brief Enable filter(0 for can1 and 14 for can2)滤波器初始化函数
 * @param None
 * @return None
 * @attention None
 */
void CANFilter_Enable(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef filter1;
	CAN_FilterTypeDef filter2;
	if(hcan->Instance == CAN1)
	{
		filter1.FilterActivation=ENABLE;
		filter1.FilterBank=0U;
		filter1.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter1.FilterIdHigh=0x0000;
		filter1.FilterIdLow=0x0000;
		filter1.FilterMaskIdHigh=0x0000;
		filter1.FilterMaskIdLow=0x0000;
		filter1.FilterMode=CAN_FILTERMODE_IDMASK;
		filter1.FilterScale=CAN_FILTERSCALE_32BIT;
		filter1.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan1,&filter1);
	}
	if(hcan->Instance == CAN2)
	{
		filter2.FilterActivation=ENABLE;
		filter2.FilterBank=14;
		filter2.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter2.FilterIdHigh=0x0000;
		filter2.FilterIdLow=0x0000;
		filter2.FilterMaskIdHigh=0x0000;
		filter2.FilterMaskIdLow=0x0000;
		filter2.FilterMode=CAN_FILTERMODE_IDMASK;
		filter2.FilterScale=CAN_FILTERSCALE_32BIT;
		filter2.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan2,&filter2);
	}
	
}
/**
 * @brief Send the message by Can（can发送函数）
 * @param None
 * @return None
 * @attention None
 */
void Can_SendMsg(CAN_HandleTypeDef *hcan,uint32_t id,int16_t data1,int16_t data2,int16_t data3,int16_t data4)
{	
	CAN_TxHeaderTypeDef Txmsg1;
	uint8_t TxData1[8];
	
	Txmsg1.DLC=0x08;
	Txmsg1.IDE=CAN_ID_STD;
	Txmsg1.RTR=CAN_RTR_DATA;
	Txmsg1.StdId=id;
	
	TxData1[0]=(unsigned char)(data1>>8);
	TxData1[1]=(unsigned char)(data1);
	TxData1[2]=(unsigned char)(data2>>8);
	TxData1[3]=(unsigned char)(data2);
	TxData1[4]=(unsigned char)(data3>>8);
	TxData1[5]=(unsigned char)(data3);
	TxData1[6]=(unsigned char)(data4>>8);
	TxData1[7]=(unsigned char)(data4);
	
	HAL_CAN_AddTxMessage(hcan,&Txmsg1,TxData1,(uint32_t *)CAN_TX_MAILBOX0);
	
}
/**
 * @brief Send the message by Can bytes（can发送函数）
 * @param None
 * @return None
 * @attention None
 */
void Can_SendMsg_by_byte(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t data[8])
{
	CAN_TxHeaderTypeDef Txmsg1;
	uint8_t TxData1[8];
	
	Txmsg1.DLC=0x08;
	Txmsg1.IDE=CAN_ID_STD;
	Txmsg1.RTR=CAN_RTR_DATA;
	Txmsg1.StdId=id;
	
	TxData1[0]=data[0];
	TxData1[1]=data[1];
	TxData1[2]=data[2];
	TxData1[3]=data[3];
	TxData1[4]=data[4];
	TxData1[5]=data[5];
	TxData1[6]=data[6];
	TxData1[7]=data[7];
	
	HAL_CAN_AddTxMessage(hcan,&Txmsg1,TxData1,(uint32_t *)CAN_TX_MAILBOX0);
}

