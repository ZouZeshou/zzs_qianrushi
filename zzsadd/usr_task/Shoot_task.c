#include "shoot_task.h"
#include "shoot.h"
#include "drv_can.h"
#include "drv_flash.h"
void StartTask04(void const * argument)
{
	portTickType task_start_time;
  for(;;)
  {		
		task_start_time = xTaskGetTickCount();
		taskENTER_CRITICAL();
		shoot_pid_param_reset();//for debug
		switch_shoot_mode(&g_shoot_mode);
		shoot_ctrl();
		calculate_single_pid_current(&s_fric_l_motor,&s_fric_l_spd_pid);
		calculate_single_pid_current(&s_fric_r_motor,&s_fric_r_spd_pid);
		calculate_serial_pid_current(&s_trans_motor,\
																 &s_trans_pos_pid,\
																 &s_trans_spd_pid);
		if(s_infantry.mode == START_GAMING)
		{
			Can_SendMsg(&hcan1,0x200,s_fric_l_motor.out_current,\
														 s_fric_r_motor.out_current,\
														 s_trans_motor.out_current,0);
		}
		else
		{
			Can_SendMsg(&hcan1,0x200,0,0,0,0);
		}	
		taskEXIT_CRITICAL();
    osDelayUntil(&task_start_time,5);
  }
}
