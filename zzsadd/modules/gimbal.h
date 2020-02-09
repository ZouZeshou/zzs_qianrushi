#ifndef GIMBAL_H
#define GIMBAL_H
#include "global.h"
#include "pid.h"
extern pid_t s_yaw_pos_pid;
extern pid_t s_yaw_spd_pid;
extern pid_t s_pitch_pos_pid;
extern pid_t s_pitch_spd_pid;
extern s_motor_data_t s_yaw_motor ;
extern s_motor_data_t s_pitch_motor;
extern pid_t s_yaw_vision_spd_pid;
extern pid_t s_pitch_vision_spd_pid;
void gimbal_param_init(void);
void gimbal_pid_param_reset(void);
void give_initial_tar(void);
void switch_gimbal_mode(uint8_t *gimbal_mode,\
												uint8_t vision_state,\
												s_motor_data_t *pitch,\
												s_motor_data_t *yaw);
void gimbal_ctrl(void);
void get_gimbal_target_pos(s_motor_data_t *s_yaw,s_motor_data_t *s_pitch);
void calculate_serial_pid_current_by_gyro(s_motor_data_t *s_motor,\
									pid_t *s_pos_pid,\
									pid_t *s_spd_pid);
#endif
