#ifndef _RAMP_H_
#define _RAMP_H_

#include "stm32f4xx_hal.h"

typedef struct{
	int32_t count;
	int32_t scale;
	float out;
}ramp_t;

#define RAMP_DEFAULT_INIT \
{ \
	.count = 30, \
	.scale = 300, \
	.out = 0,\
		} \

#define RAMP_FAST_INIT \
{ \
	.count = 30, \
	.scale = 150, \
	.out = 0,\
		} \

void ramp_init(ramp_t *ramp);
float ramp_cal(ramp_t *ramp);
extern ramp_t ramp_x,ramp_y;

#endif
