#ifndef __KEYBOARD_H
#define __KEYBOARD_H
#include "global.h"
#include "drv_dbus.h"
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
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

#endif
