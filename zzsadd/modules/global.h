#ifndef __GLOBAL_H
#define __GLOBAL_H
#include "stm32f4xx.h"
#include "stdint.h"
#include "drv_dbus.h"
/***************** debug *********************/
#define NO_JUDGE_SYSTEM 1
/***************** for robot *****************/
#define ROBOT_ID  1
/************ rc and key constant **********************/
#define MOVE_WSCONST  8000
#define MOVE_ADCONST  6000
#define SHIFT_WSCONST 5000
#define SHIFT_ADCONST 3000
#define Q_WSCONST     5000
#define Q_ADCONST     3000
#define E_WSCONST 	  3000
#define E_ADCONST     2000
#define CHANNEL_X_CONST 12.12f
#define CHANNEL_Y_CONST 12.12f
#define CHANNEL_YAW_CONST -0.048f
#define CHANNEL_PITCH_CONST -0.012f
#define MOUSE_YAW_CONST -0.8f
#define MOUSE_PITCH_CONST -0.2f
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
#define  CHASSIS_MAX_SPEED     8000
#define  SCALE_V_W             0.35f 
#define  ROTATE_SPD            5000
#define	 C_FOLLOW              0
#define	 C_ROTATE              1
#define  C_SWING               2
#define	 C_LOCK                3
#define  C_NO_CAP              4
#define	 C_USE_CAP             5
extern uint8_t g_chassis_move_mode ;
extern uint8_t g_chassis_power_mode ;
/*****definition of gimbal ********************/
#define YAWMID                   2700
#define PITCHMID                 7500
#define ENCODER_SPD_TO_DPS       5.7608f
#define	G_MANUAL                 0
#define	G_AUTO									 1
#define G_LOCK                   2
#define	G_ENCODER                3
#define	G_GYRO									 4 
extern uint8_t g_gimbal_move_mode ;
extern uint8_t g_gimbal_info_src ;
/*****definition of shoot ********************/
#define ANGLE_ENCODE            22.7527778f
#define TRAVEL                  29487.6f//8191*36/10
#define ENCODE_ANGLE            0.0439506f //360/8191
#define RPM_DPS                 6 //1/60*360
#define ANGLE_RAD               0.0174532f //3.1415926/180
#define RAD_ANGLE               57.29578f
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
