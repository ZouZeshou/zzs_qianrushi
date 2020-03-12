#ifndef __DRV_DBUS_H
#define __DRV_DBUS_H
#include "stdint.h"
#define UP 1
#define MID 3
#define DOWN 2

struct RC
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t l;
		uint8_t r;
		uint8_t l_press;
	}mouse;
	union
	{
			uint16_t v;
			 struct 
			{
				uint16_t W:1;
				uint16_t S:1;
				uint16_t A:1;
				uint16_t D:1;
				uint16_t SHIFT:1;
				uint16_t CTRL:1;
				uint16_t Q:1;
				uint16_t E:1;
				uint16_t R:1;
				uint16_t F:1;
				uint16_t G:1;
				uint16_t Z:1;
				uint16_t X:1;
				uint16_t C:1;
				uint16_t V:1;
				uint16_t B:1;
			} bit;
	}key;
};
typedef struct RC RC_Ctl_t;
extern RC_Ctl_t RC_Ctl;
void get_dbus_data (void);
void dubs_data_init(void);

#endif
