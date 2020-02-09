#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "infantry.h"
#include "stdbool.h"
typedef struct
{
	uint8_t err_cnt;
	bool    is_err;
}s_err_t;

typedef struct
{
  uint16_t dbus;
  uint16_t chassis[4];
  uint16_t yaw;
	uint16_t pitch;
  uint16_t trans;
	uint16_t fric_l;
	uint16_t fric_r;
	uint16_t chas_imu;//the imu in the chassis
	uint16_t pitch_imu;//the imu on the shootor
	uint16_t board_imu;//the imu in stm32f427IIHx typeA
	uint16_t judge;
	uint16_t nuc;
	uint16_t cap_board;
} s_fps_t;

typedef struct
{
	s_err_t dbus;
	s_err_t chassis[4];
	s_err_t yaw;
	s_err_t pitch;
	s_err_t trans;
	s_err_t chas_imu;
	s_err_t pitch_imu;
	s_err_t board_imu;
	s_err_t judge;
	s_err_t nuc;
}s_sys_err_t;

extern s_sys_err_t s_sys_err;
extern s_fps_t s_fps;

void show_fps_data(void);
void calculate_fps_per_second(uint16_t delay_value, s_fps_t *fps);
void judge_whether_module_offline(s_fps_t fps,s_sys_err_t *sys_err);
void judge_one_offline(int fps,\
											 s_err_t *err,\
											 int offline_threshold,\
											 uint8_t offline_count);
void Sys_Running_Well(void);
void Sys_initializing(void);
void SysModule_Offline(void);
void Sys_Alarming(void);
#endif
