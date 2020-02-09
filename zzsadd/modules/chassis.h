#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "global.h"
#include "pid.h"
#include "drv_dbus.h"
#include "drv_keyboard.h"
typedef struct
{
	int16_t max_power;/* the maximum power expected */
	int16_t max_spd;/* the maximum speed of four wheel*/
	uint16_t current_sum;/* the maximum current sum which will be send to motor*/
	uint16_t max_cur_sum;
	uint16_t min_cur_sum;
	float power_spd_scale;/* a constant show the relation of backspeed and power*/
	float spd_cur_scale;/* a constant show the relation of current sum and power*/ 
}s_power_control_t;
typedef struct
{
	s_power_control_t power_ctrl;
	float gyro_spd;/*the gyro speed set in chassis*/
	float gyro_angle;/*the gyro angle set in chassis*/
	float angle_diff;/*the angle difference to gimbal(chassis-gimbal)*/
	int16_t gim_Vx;/*the speed from gimbal view (forward--positive)*/
	int16_t gim_Vy;/*the speed from gimbal view (right--positive)*/
	int16_t gim_W;/*the speed from gimbal view (CCW--positive)*/
	int16_t Vx;/*the speed from chassis view (forward--positive)*/
	int16_t Vy;/*the speed from chassis view (right--positive)*/
	int16_t W;/*the speed from chassis view (CCW--positive)*/
}s_chassis_t;

extern pid_t s_chassis_spd_pid[4];
extern pid_t s_follow_pos_pid;
extern pid_t s_follow_spd_pid;
extern pid_t s_swing_pid ;
extern s_motor_data_t s_chassis_motor[4] ;
extern s_chassis_t s_chassis ;

void chassis_param_init(void);
void chassis_pid_param_reset(void);
void get_spd_from_keyboard(RC_Ctl_t s_rc,s_chassis_t *s_chas);
void switch_chassis_mode(uint8_t *chassis_mode,\
												 s_keymouse_t s_key,\
												 s_chassis_t *s_chas,\
												 s_motor_data_t s_yaw);
void transform_chassis_spd(s_chassis_t *s_chas);
void get_chassis_spd(s_chassis_t *s_chas,\
										 s_motor_data_t *s_motor_0,\
										 s_motor_data_t *s_motor_1,\
										 s_motor_data_t *s_motor_2,\
										 s_motor_data_t *s_motor_3);
void limit_chassis_current(float max_value,int16_t *cur_1,int16_t *cur_2,int16_t *cur_3,int16_t *cur_4);
void calculate_power_param(s_power_control_t *s_power,int16_t spd1,int16_t spd2,int16_t spd3,int16_t spd4);
#endif
