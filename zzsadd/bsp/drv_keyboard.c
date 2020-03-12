#include "drv_keyboard.h"
s_keymouse_t s_keymouse;

/**
 * @brief deal data from key and mouse
 * @param *s_key -- the key mouse data(out)
 * @param s_rc -- the remote data(in)
 * @return KeyMousedata
 * @attention None
 */
void get_keymouse_data(s_keymouse_t *s_key,RC_Ctl_t s_rc,uint16_t long_time)
{
	get_key_state(&s_key->Q_state,s_rc.key.bit.Q,&s_key->cnt.Q_cnt,long_time);
	get_key_state(&s_key->W_state,s_rc.key.bit.W,&s_key->cnt.W_cnt,long_time);
	get_key_state(&s_key->E_state,s_rc.key.bit.E,&s_key->cnt.E_cnt,long_time);
	get_key_state(&s_key->R_state,s_rc.key.bit.R,&s_key->cnt.R_cnt,long_time);
	get_key_state(&s_key->A_state,s_rc.key.bit.A,&s_key->cnt.A_cnt,long_time);
	get_key_state(&s_key->S_state,s_rc.key.bit.S,&s_key->cnt.S_cnt,long_time);
	get_key_state(&s_key->D_state,s_rc.key.bit.D,&s_key->cnt.D_cnt,long_time);
	get_key_state(&s_key->F_state,s_rc.key.bit.F,&s_key->cnt.F_cnt,long_time);
	get_key_state(&s_key->Z_state,s_rc.key.bit.Z,&s_key->cnt.Z_cnt,long_time);
	get_key_state(&s_key->X_state,s_rc.key.bit.X,&s_key->cnt.X_cnt,long_time);
	get_key_state(&s_key->C_state,s_rc.key.bit.C,&s_key->cnt.C_cnt,long_time);
	get_key_state(&s_key->V_state,s_rc.key.bit.V,&s_key->cnt.V_cnt,long_time);
	get_key_state(&s_key->Shift_state,s_rc.key.bit.SHIFT,&s_key->cnt.Shift_cnt,long_time);
	get_key_state(&s_key->Ctrl_state,s_rc.key.bit.CTRL,&s_key->cnt.Ctrl_cnt,long_time);
	get_key_state(&s_key->Mouse_l_state,s_rc.mouse.l,&s_key->cnt.Mouse_l_cnt,long_time);
	get_key_state(&s_key->Mouse_r_state,s_rc.mouse.r,&s_key->cnt.Mouse_r_cnt,long_time);
	
}
/**
 * @brief get the key state from key value
 * @param *key_state -- the key state data(out)
 * @param key_value -- the value of key(in)
 * @param *cnt -- the count value to judge if key is long pressed(in)
 * @param long_time -- a constant value to compare (in)
 * @return None
 * @attention None
 */
void get_key_state(e_kb_state_t *sta,\
									 uint8_t key,\
									 uint16_t *cnt,\
									 uint16_t long_time)
{
	switch (*sta)
  {
    case KEY_RELEASE:
    {
      if (key)
        *sta = KEY_WAIT_EFFECTIVE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:
    {
      if (key)
        *sta = KEY_PRESS_ONCE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    
    case KEY_PRESS_ONCE:
    {
      if (key)
      {
        *sta = KEY_PRESS_DOWN;
				*cnt = 0;
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:
    {
      if (key)
      {
         if (*cnt++ > long_time)
            *sta = KEY_PRESS_LONG;
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:
    {
      if (!key)
      {
        *sta = KEY_RELEASE;
      }
    }break;
    
    default:
    break;
      
  }
}
