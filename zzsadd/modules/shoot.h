#ifndef __SHOOT_H
#define __SHOOT_H
#include "global.h"
#include "pid.h"
#include "stdbool.h"
extern pid_t s_trans_pos_pid;
extern pid_t s_trans_spd_pid;
extern pid_t s_fric_l_spd_pid;
extern pid_t s_fric_r_spd_pid;
extern s_motor_data_t s_trans_motor;
extern s_motor_data_t s_fric_l_motor;
extern s_motor_data_t s_fric_r_motor;
void shoot_param_init(void);
void shoot_pid_param_reset(void);
void switch_shoot_mode(uint8_t *shoot_mode);
void shoot_ctrl(void);
void get_trans_target_pos(s_motor_data_t *s_motor);
void manual_shoot_ctrl(s_motor_data_t *s_fric_l,s_motor_data_t *s_fric_r);
void continue_motor_pos(s_motor_data_t *s_motor);
void calculate_single_pid_current(s_motor_data_t *s_motor,pid_t *s_spd_pid);
void calculate_serial_pid_current(s_motor_data_t *s_motor,\
									pid_t *s_pos_pid,\
									pid_t *s_spd_pid);
void get_real_spd(uint8_t level,s_motor_data_t * s_fric_l);
void get_fric_spd_from_realspd(float real_spd,\
															 s_motor_data_t * s_fric_l,\
															 s_motor_data_t * s_fric_r);
int shoot_once(void);
int Just_Fire(bool is_shoot, uint8_t time_diff);
int shoot_by_heat(uint16_t *energy,uint8_t heat_reduce,uint8_t max_heat);
#endif
