#ifndef __INFANTRY_H
#define __INFANTRY_H
#include "stdint.h"
#include "stdbool.h"
typedef enum
{
	INFANTRY_INITIALIZING = 0,
	INFANTRY_RUN_WELL = 1,
	INFANTRY_ALARM = 2
}e_sys_state;

typedef enum
{
	START_GAMING = 0,
  ADD_BULLET,
	HAVE_BULLET,
	RELAX,
	CALIBRATE,
}e_sys_mode;

typedef enum
{
  GIMBAL_RELAX         = 0,
	GIMBAL_NORMAL        = 1,
  GIMBAL_ERROR         = 2,
}e_gimbal_state;

typedef struct
{
	e_sys_state     state;
	e_sys_mode   		mode;
	uint16_t        add_bullet_cnt;
	bool						lock_gim_pos;
	bool						lock_gim_target;
	uint16_t				cnt;
	uint8_t 				is_open_magazine;
	uint8_t					is_turn_on_laser;
}s_infantry_t;

extern s_infantry_t s_infantry;
void infantry_init(void);
void rc_mode_switch(void);
#endif
