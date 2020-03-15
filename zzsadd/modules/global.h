#ifndef __GLOBAL_H
#define __GLOBAL_H
#include "stm32f4xx.h"
#include "stdint.h"
#include "key.h"
#include "periphetal.h"
#include "parameter.h"
/*****************global struct*******************/
typedef struct 
{
	int     id;
	uint8_t is_pos_ready;
	float   gyro_speed;
	float   gyro_angle;
	int16_t anglespd_times_ago;
	int     reduction_ratio;
	int     frequency;
	int16_t mid_pos;
	int16_t max_pos;
	int16_t min_pos;
	int16_t temperature;
	int16_t back_speed;
	int16_t back_position;
	int16_t back_pos_last;
	int64_t circle_num;
	float   target_speed;
	int     real_spd;
	int64_t tol_pos;
	double  target_pos;
	float   target_ang;
	int16_t out_current;
	int16_t back_current;
}s_motor_data_t ;
typedef struct 
{
	uint8_t  real_id;
	uint16_t real_heat;
	uint16_t heat_reduce;
	uint16_t max_heat;
	uint16_t max_spd;
	
	uint16_t max_power;
	uint16_t real_power;
	uint16_t power_buffer;
}s_judgesystem_t;
extern s_judgesystem_t  s_judge;
/*****definition of chassis*******************/
#define	 C_FOLLOW              0
#define	 C_ROTATE              1
#define  C_SWING               2
#define	 C_LOCK                3
#define  C_NO_CAP              4
#define	 C_USE_CAP             5
extern uint8_t g_chassis_move_mode ;
extern uint8_t g_chassis_power_mode ;
/*****definition of gimbal ********************/
#define	G_MANUAL                 0
#define	G_AUTO									 1
#define G_LOCK                   2
#define	G_ENCODER                3
#define	G_GYRO									 4 
extern uint8_t g_gimbal_move_mode ;
extern uint8_t g_gimbal_info_src ;
/*****definition of shoot ********************/
#define S_MANUAL  							0
#define	S_AUTO 									1
#define S_CLEAR_BULLETS         2
#define S_JUST_SHOOT            3
extern uint8_t g_shoot_mode ;
/****definition of vision***************/
#define V_NOT_USE          0
#define V_BIG_FAN          1
#define V_SMALL_FAN        2
#define V_ROBOT            3
#define V_GYRO             4
#define V_ABNORMAL         5
#define V_LOSE             6
#define V_CATCH    	       7
extern uint8_t g_vision_mode ;
extern uint8_t g_vision_state ;

#endif
