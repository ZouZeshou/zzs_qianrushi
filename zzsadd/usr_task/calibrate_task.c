#include "calibrate_task.h"
#include "drv_imu.h"
#include "drv_flash.h"
#include "infantry.h"
#include "drv_dbus.h"
#define RC_CALI_VALUE_HOLE 600
uint8_t cali_flag = 0;
void StartTask08(void const * argument)
{
	static uint16_t rc_cmd_time = 0;
  for(;;)
  {
		if((RC_Ctl.rc.ch0 - 1024) < -RC_CALI_VALUE_HOLE &&\
			 (RC_Ctl.rc.ch1 - 1024) < -RC_CALI_VALUE_HOLE &&\
			 (RC_Ctl.rc.ch2 - 1024) > RC_CALI_VALUE_HOLE &&\
			 (RC_Ctl.rc.ch3 - 1024) < -RC_CALI_VALUE_HOLE &&\
				RC_Ctl.rc.s1==UP && RC_Ctl.rc.s2==MID)
		{//两个摇杆打成 \../,保持2s
      if(rc_cmd_time++ > 2000/5)
			{
				cali_flag = 1;
				rc_cmd_time = 0;
			}
    }
		else
		{
			rc_cmd_time = 0;
		}
		if(cali_flag)
		{
			s_infantry.mode = CALIBRATE;
			get_imu_cali_data();
			taskENTER_CRITICAL();
			save_cali_data();
			taskEXIT_CRITICAL();
			cali_flag = 0;
		}
		else
			s_infantry.mode = START_GAMING;
    osDelay(5);
  }

}
