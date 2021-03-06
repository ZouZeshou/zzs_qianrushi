/**
 * left is s1
 * right is s2
 * top and right is the positive direction of channel
 */
#include "drv_dbus.h"
#include "monitor_task.h"
#include "usart.h"
#include "drv_uart.h"
#include "stdlib.h"
#include "string.h"
RC_Ctl_t RC_Ctl = {0};
/**
 * @brief Receive data from remote device
 * @param *s_rc-- the remote data
 * @return *dbus_buffer --the data dbus_buffer address from uart
 * @attention  None
 */
void dubs_data_init(void)
{
	memset(&RC_Ctl,0,sizeof(RC_Ctl_t));
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;
	RC_Ctl.rc.s1 = DOWN;
	RC_Ctl.rc.s2 = MID;
}
/**
 * @brief Receive data from remote device
 * @param *s_rc-- the remote data
 * @return *dbus_buffer --the data buff address from uart
 * @attention  None
 */
void get_dbus_data (void)
{
	static uint16_t cnt_l = 0;
	static int16_t last_mouse_y = 0;
	s_fps.dbus++;
	if(dbus_buffer[12] < 0x02 && dbus_buffer[13] < 0x02)
	{
		RC_Ctl.rc.ch0 = (dbus_buffer[0]| (dbus_buffer[1] << 8)) & 0x07ff; //!< Channel 0
		RC_Ctl.rc.ch1 = ((dbus_buffer[1] >> 3) | (dbus_buffer[2] << 5)) & 0x07ff; //!< Channel 1
		RC_Ctl.rc.ch2 = ((dbus_buffer[2] >> 6) | (dbus_buffer[3] << 2) | (dbus_buffer[4] << 10)) & 0x07ff;//!< Channel 2
		RC_Ctl.rc.ch3 = ((dbus_buffer[4] >> 1) | (dbus_buffer[5] << 7)) & 0x07ff; //!< Channel 3
		RC_Ctl.rc.s1 = ((dbus_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
		RC_Ctl.rc.s2 = ((dbus_buffer[5] >> 4)& 0x0003); //!< Switch right9 / 9
		RC_Ctl.mouse.x = dbus_buffer[6] | (dbus_buffer[7] << 8); //!< Mouse X axis
		RC_Ctl.mouse.y = dbus_buffer[8] | (dbus_buffer[9] << 8); //!< Mouse Y axis
		RC_Ctl.mouse.z = dbus_buffer[10] | (dbus_buffer[11] << 8); //!< Mouse Z axis
		RC_Ctl.mouse.l = dbus_buffer[12]; //!< Mouse Left Is Press ?
		RC_Ctl.mouse.r = dbus_buffer[13]; //!< Mouse Right Is Press ?
		RC_Ctl.key.v = dbus_buffer[14] | (dbus_buffer[15] << 8); //!< KeyBoard value
		RC_Ctl.rc.ch0 = (abs(RC_Ctl.rc.ch0 - 1024) > 10 ? RC_Ctl.rc.ch0 : 1024);
		RC_Ctl.rc.ch1 = (abs(RC_Ctl.rc.ch1 - 1024) > 10 ? RC_Ctl.rc.ch1 : 1024);
		RC_Ctl.rc.ch2 = (abs(RC_Ctl.rc.ch2 - 1024) > 10 ? RC_Ctl.rc.ch2 : 1024);
		RC_Ctl.rc.ch3 = (abs(RC_Ctl.rc.ch3 - 1024) > 10 ? RC_Ctl.rc.ch3 : 1024);
		/*******deal with mouse.y*************/
		if(abs(RC_Ctl.mouse.y - last_mouse_y) > 150)
			RC_Ctl.mouse.y = last_mouse_y;
		static uint16_t same_value_cnt = 0;
		if(RC_Ctl.mouse.y == last_mouse_y && RC_Ctl.mouse.y != 0)
			same_value_cnt ++;
		if(same_value_cnt > 50)
		{
			same_value_cnt = 0;
			RC_Ctl.mouse.y = 0;
		}
		last_mouse_y = RC_Ctl.mouse.y;
		/**************deal with mouse.l is really pressed*********/  
		if(RC_Ctl.mouse.l == 1)
			cnt_l ++;
		else 
			cnt_l = 0;
		
		if(cnt_l > 6)
			RC_Ctl.mouse.l_press = 1;
		else
			RC_Ctl.mouse.l_press = 0;
	}
}

