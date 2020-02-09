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
/*************** key funtion define************/
//      direction  key
#define FORWARD_NORMAL    	 (RC_Ctl.key.bit.W)
#define FORWARD_SLOWLY    	 (RC_Ctl.key.bit.Q)
#define FORWARD_VERY_SLOWLY  (RC_Ctl.key.bit.E)
#define BACK        				 (RC_Ctl.key.bit.S)
#define LEFT       					 (RC_Ctl.key.bit.A)
#define RIGHT      					 (RC_Ctl.key.bit.D)
#define LEFT_SLOWLY					 (RC_Ctl.key.bit.Z)
#define RIGHT_SLOWLY				 (RC_Ctl.key.bit.X)
//      speed      key
#define FAST_SPD   					 (RC_Ctl.key.bit.SHIFT)

//      function   key or mouse operate	
#define OPEN_MAGAZINE				 (RC_Ctl.key.bit.R)
#define CLOSE_MAGAZINE       (RC_Ctl.key.bit.SHIFT && RC_Ctl.key.bit.R)
#define	SHIFT_CTRL	   			 (RC_Ctl.key.bit.SHIFT)
#define MANUAL_ATTACK				 (RC_Ctl.mouse.r)
#define MANUAL_SHOOT				 (RC_Ctl.mouse.l_press)
#define ROBOT_ATTACK         (RC_Ctl.key.bit.Z)
#define GYRO_ATTACK					 (RC_Ctl.key.bit.X)
#define FANWHEEL_ATTACK      (RC_Ctl.key.bit.C)
#define GIMBAL_LOCK          (RC_Ctl.key.bit.F)
#define GIMBAL_UNLOCK        (RC_Ctl.key.bit.SHIFT && RC_Ctl.key.bit.F)
#define CHASSIS_SW           (RC_Ctl.key.bit.V) 
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
#endif
