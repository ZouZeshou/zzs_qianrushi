#include "chassis_task.h"
#include "chassis.h"
#include "drv_dbus.h"
#include "drv_keyboard.h"
#include "gimbal.h"
#include "shoot.h"
#include "drv_can.h"

void StartTask03(void const * argument)
{
	portTickType task_start_time;
  for(;;)
  {
		task_start_time = xTaskGetTickCount();
		taskENTER_CRITICAL();
		chassis_pid_param_reset();/*for debug*/
		switch_chassis_mode(&g_chassis_move_mode,s_keymouse,&s_chassis,s_yaw_motor);
		get_spd_from_keyboard(RC_Ctl,&s_chassis);
		calculate_power_param(&s_chassis.power_ctrl,\
													s_chassis_motor[0].back_speed,\
													s_chassis_motor[1].back_speed,\
													s_chassis_motor[2].back_speed,\
													s_chassis_motor[3].back_speed);
		transform_chassis_spd(&s_chassis);
		get_chassis_spd(&s_chassis,\
										&s_chassis_motor[0],\
										&s_chassis_motor[1],\
										&s_chassis_motor[2],\
										&s_chassis_motor[3]);
		for(int i=0;i<4;i++)
		{
			calculate_single_pid_current(&s_chassis_motor[i],&s_chassis_spd_pid[i]);
		}
		limit_chassis_current(s_chassis.power_ctrl.current_sum,\
																	 &s_chassis_motor[0].out_current,\
																	 &s_chassis_motor[1].out_current,\
																	 &s_chassis_motor[2].out_current,\
																	 &s_chassis_motor[3].out_current);
		if(s_infantry.mode == NORMAL)
		{
			Can_SendMsg(&hcan2,0x200,s_chassis_motor[0].out_current,\
														 s_chassis_motor[1].out_current,\
														 s_chassis_motor[2].out_current,\
														 s_chassis_motor[3].out_current);
		}
		else
		{
			Can_SendMsg(&hcan2,0x200,0,0,0,0);
		}
		uint8_t cap_data[8]={WANT_USE_CAP,0,0,0,0,0,0,0};
		Can_SendMsg_by_byte(&hcan2,0x199,cap_data);
		taskEXIT_CRITICAL();
    osDelayUntil(&task_start_time,5);
  }
}

