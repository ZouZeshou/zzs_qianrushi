#ifndef __INFANTRY_H
#define __INFANTRY_H
#include "stdint.h"
#include "stdbool.h"
typedef enum
{
	INFANTRY_INITIALIZING = 0,
	INFANTRY_RUN_WELL = 1,
	INFANTRY_ALARM = 2,	
}e_sys_state;

typedef enum
{
	NORMAL = 0,
	NO_FORCE = 1,
	CALIBRATE,
}e_sys_mode;

typedef struct
{
	e_sys_state     state;
	e_sys_mode   		mode;
}s_infantry_t;

extern s_infantry_t s_infantry;
void infantry_init(void);
void rc_mode_switch(void);
#endif
