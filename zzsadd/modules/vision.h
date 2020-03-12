#ifndef __VISION_H
#define __VISION_H
#include "global.h"
#include "pid.h"
#include "drv_nuc_interface.h"

extern pid_t  s_yaw_track_pid;
extern pid_t  s_pit_track_pid;
extern pid_t  s_yaw_fanwheel_pid;
extern pid_t  s_pitch_fanwheel_pid;

void vision_param_init(void);
void detect_vision_state(uint8_t *vision_state);
void switch_vision_mode(uint8_t *e_vision_mode);
void vision_pid_pamar_reset(void);
void vision_shoot_ctrl(s_vision_t *vision);
void get_target_by_vision(s_vision_t * vision);
uint8_t get_pitch_compensate_angle(s_vision_t * vision,s_motor_data_t *pitch);
void get_yaw_err_from_vision(s_vision_t *vision);
void get_pitch_err_from_vision(s_vision_t *vision);
void fanwheel_shoot_ctr(s_vision_t *vision);
void robot_shoot_ctr(s_vision_t *vision);
void gyro_shoot_ctr(s_vision_t *vision);
float get_robot_pitch_comp(float dist, float initial_angle, float bullet_speed);
float get_robot_pitch_initial_angle(void);
float get_fan_y_target(void);
float get_fan_pitch_comp(float dist, float tvec_y, float bullet_spd);
#endif
