/*******************************************************************************/
/** @file 		judge_system.c
 *  @version  1.0
 *  @date 		2019.3.27
 *  @brief 		about the data transmit with the judge system 
 *  @author 	Echo
 */
#include "drv_judgesystem.h"
#include "drv_can.h"
#include "string.h"
#include "global.h"
data_to_judge_t															ToJudge_Data;

ext_game_state_t 														Judge_GameState;
ext_game_result_t     											Judge_GameResult;
ext_game_robot_survivors_t 									Judge_GameRobotSurvivors;
ext_event_data_t 														Judge_EventData;
ext_supply_projectile_action_t     					Judge_SupplyProjectileAction;
ext_supply_projectile_booking_t 						Judge_SupplyProjectileBooking;

ext_game_robot_state_t											Judge_GameRobotState;
ext_power_heat_data_t												Judge_PowerHeatData;
ext_game_robot_pos_t												Judge_GameRobotPos;
ext_buff_musk_t															Judge_BuffMusk;
ext_robot_hurt_t														Judge_RobotHurt;
ext_shoot_data_t  													Judge_ShootData;
ext_student_interactive_header_data_t				Judge_StudentInfoHeader;
client_custom_data_t												Judge_ClientData;
robot_interactive_data_t  									Judge_RobotInterfaceData;

void judgesystem_param_init(void)
{
	memset(&Judge_GameState,0,sizeof(ext_game_state_t));
	memset(&Judge_GameRobotState,0,sizeof(ext_game_robot_state_t));
	memset(&Judge_PowerHeatData,0,sizeof(ext_power_heat_data_t));
	memset(&Judge_GameRobotPos,0,sizeof(ext_game_robot_pos_t));
	memset(&Judge_BuffMusk,0,sizeof(ext_buff_musk_t));
	memset(&Judge_RobotHurt,0,sizeof(ext_robot_hurt_t));
	memset(&Judge_ShootData,0,sizeof(ext_shoot_data_t));
	memset(&Judge_StudentInfoHeader,0,sizeof(ext_student_interactive_header_data_t));
	memset(&Judge_ClientData,0,sizeof(client_custom_data_t));
	if(NO_JUDGE_SYSTEM)
	{
		s_judge.real_id = 3;
		s_judge.real_heat = 0;
		s_judge.max_power = 80;
		s_judge.max_spd = 30;
		s_judge.power_buffer = 60;
	}
}

/* send the data to the judge-board by using the can-bus, 
 * then the judge-board will send them to the judge by usart */
void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask)
{
	wl4data w4data;
	uint8_t candata[8];
	/* data send */
	w4data.f 	= data1;
	candata[0] = w4data.c[0];
	candata[1] = w4data.c[1];
	candata[2] = w4data.c[2];
	candata[3] = w4data.c[3];
	w4data.f 	= data2;
	candata[4] = w4data.c[0];
	candata[5] = w4data.c[1];
	candata[6] = w4data.c[2];
	candata[7] = w4data.c[3];
	Can_SendMsg_by_byte(&hcan1,0x300,candata);		
	w4data.f 	= data3;
	candata[0] = w4data.c[0];
	candata[1] = w4data.c[1];
	candata[2] = w4data.c[2];
	candata[3] = w4data.c[3];
	candata[4] 	= mask;
	memset(&candata[5],0,3);
	Can_SendMsg_by_byte(&hcan1,0x301,candata);		
}
