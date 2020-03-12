#ifndef __KEYBOARD_H
#define __KEYBOARD_H
#include "global.h"
#include "drv_dbus.h"

#define Q_X 0
#define Q_Y 5000
#define E_X 0
#define E_Y 2000
#define SHIFT_X 5000
#define SHIFT_Y 3000
#define MOVE_X 6000
#define MOVE_Y 8000


typedef enum
{
  KEY_RELEASE = 0,
	KEY_WAIT_EFFECTIVE,
  KEY_PRESS_ONCE,
  KEY_PRESS_DOWN,
  KEY_PRESS_LONG
} e_kb_state_t;

typedef struct
{
	e_kb_state_t Q_state;
	e_kb_state_t W_state;
	e_kb_state_t E_state;
	e_kb_state_t R_state;
	e_kb_state_t A_state;
	e_kb_state_t S_state;
	e_kb_state_t D_state;
	e_kb_state_t F_state;
	e_kb_state_t Z_state;
	e_kb_state_t X_state;
	e_kb_state_t C_state;
	e_kb_state_t V_state;
	e_kb_state_t G_state;
	e_kb_state_t B_state;
	e_kb_state_t Shift_state;
	e_kb_state_t Ctrl_state;
	e_kb_state_t Mouse_l_state;
	e_kb_state_t Mouse_r_state;
	struct
	{
		uint16_t Q_cnt;
		uint16_t W_cnt;
		uint16_t E_cnt;
		uint16_t R_cnt;
		uint16_t A_cnt;
		uint16_t S_cnt;
		uint16_t D_cnt;
		uint16_t F_cnt;
		uint16_t Z_cnt;
		uint16_t X_cnt;
		uint16_t C_cnt;
		uint16_t V_cnt;
		uint16_t G_cnt;
		uint16_t B_cnt;
		uint16_t Shift_cnt;
		uint16_t Ctrl_cnt;
		uint16_t Mouse_l_cnt;
		uint16_t Mouse_r_cnt;
	}cnt;	
}s_keymouse_t;
extern s_keymouse_t s_keymouse;
void get_keymouse_data(s_keymouse_t *s_key,RC_Ctl_t s_rc,uint16_t long_time);
void get_key_state(e_kb_state_t *key_state,\
									 uint8_t key_value,\
									 uint16_t *cnt,\
									 uint16_t long_time);
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
#define SMALL_FAN_ATTACK		 (RC_Ctl.key.bit.X)
#define BIG_FAN_ATTACK       (RC_Ctl.key.bit.C)
#define GIMBAL_LOCK_SW       (s_keymouse.F_state == KEY_PRESS_ONCE)
#define CHASSIS__GYRO_SW     (s_keymouse.V_state == KEY_PRESS_ONCE) 
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
#endif
