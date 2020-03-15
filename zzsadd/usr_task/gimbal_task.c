#include "gimbal_task.h"
#include "gimbal.h"
#include "vision.h"
#include "drv_dbus.h"
#include "drv_keyboard.h"
#include "shoot.h"
#include "drv_can.h"
#include "iwdg.h"
void StartTask02(void const * argument)
{
	portTickType task_start_time;
  for(;;)
  {	
		task_start_time = xTaskGetTickCount();
		taskENTER_CRITICAL();
		/**** global task******/
		get_keymouse_data(&s_keymouse,RC_Ctl,LONG_PRESS_TIME/5);
		rc_mode_switch();
		/**** vision task******/
		vision_pid_pamar_reset();
		detect_vision_state(&g_vision_state);
		switch_vision_mode(&g_vision_mode);
		/**** gimbal task******/
		gimbal_pid_param_reset();/*for debug*/
		switch_gimbal_mode(&g_gimbal_move_mode,g_vision_state,&s_pitch_motor,&s_yaw_motor);
		gimbal_ctrl();
		if(s_infantry.mode == NORMAL)
		{
			Can_SendMsg(&hcan1,0x1FF,0,s_pitch_motor.out_current,0,0);
			Can_SendMsg(&hcan2,0x1FF,s_yaw_motor.out_current + s_trans_motor.out_current*TRANS_COMP_CONST,0,0,0);
		}
		else
		{
			Can_SendMsg(&hcan1,0x1FF,0,0,0,0);
			Can_SendMsg(&hcan2,0x1FF,0,0,0,0);
		}
		
		HAL_IWDG_Refresh(&hiwdg);
		
		taskEXIT_CRITICAL();
    osDelayUntil(&task_start_time,5);
  }
}
