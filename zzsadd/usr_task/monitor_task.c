#include "monitor_task.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_io.h"
#include "drv_buzzer.h"
#include "infantry.h"
#include "oled.h"
#include "string.h"
#define OFFLINE_COUNT     2
#define OFFLINE_THRESHOLD1	850
#define OFFLINE_THRESHOLD2	200
#define OFFLINE_THRESHOLD3	100
s_fps_t 			s_fps={0};
s_sys_err_t 	s_sys_err={0};
/* standard value 
	1000,//chassis motor LF
	1000,//chassis motor RF
	1000,//chassis motor LD
	1000,//chassis motor RD
	1000,//yaw motor
	1000,//pitch motor
	1000,//trigger motor
	1000,//board imu
	1000,//gim imu
	1000,//chassis imu
	140, //dbus
	140, //nuc
	10   //judge
*/
void StartTask06(void const * argument)
{
  for(;;)
  {
		static uint8_t count;
		if(count++ > 7)
		{
			if(s_infantry.state != INFANTRY_INITIALIZING)
			{
				calculate_fps_per_second(200, &s_fps);
				judge_whether_module_offline(s_fps,&s_sys_err);
				//show_fps_data();
				memset(&s_fps, 0, sizeof(s_fps_t));
			}
			count = 0;
		}
		static uint16_t count1 = 0;
		count1 ++;
		if(count1 > 1)
		{
			if(s_infantry.state == INFANTRY_RUN_WELL)
				Sys_Running_Well();
			else if(s_infantry.state == INFANTRY_INITIALIZING)
				Sys_initializing();
			else if(s_infantry.state == INFANTRY_ALARM)
				SysModule_Offline();
			count1 = 0;
		}
		Sys_Alarming();

    oled_LOGO();
		 
    osDelay(25);
  }
}
/************some monitor functions **********************/
/**
 * @brief this function is to calculate the fps per second 
 * @param delay_value-- the time gap of task
 * @param *fps -- the fps data struct 
 * @return None
 * @attention None
 */
void calculate_fps_per_second(uint16_t delay_value, s_fps_t *fps)
{
	fps->board_imu  *= (1000/delay_value);
	fps->chassis[0] *= (1000/delay_value);
	fps->chassis[1] *= (1000/delay_value);
	fps->chassis[2] *= (1000/delay_value);
	fps->chassis[3] *= (1000/delay_value);
	fps->chas_imu   *= (1000/delay_value);
	fps->dbus       *= (1000/delay_value);
	fps->pitch_imu  *= (1000/delay_value);
	fps->judge      *= (1000/delay_value);
	fps->nuc        *= (1000/delay_value);
	fps->pitch      *= (1000/delay_value);
	fps->trans      *= (1000/delay_value);
	fps->yaw        *= (1000/delay_value);
	fps->cap_board  *= (1000/delay_value);
	fps->fric_l     *= (1000/delay_value);
	fps->fric_r     *= (1000/delay_value);	
}
/**
 * @brief this function is to calculate the fps per second 
 * @param delay_value-- the time gap of task
 * @param *fps -- the fps data struct 
 * @return None
 * @attention None
 */
void judge_whether_module_offline(s_fps_t fps,s_sys_err_t *sys_err)
{
	/* board imu detect */
	judge_one_offline(fps.board_imu,&sys_err->board_imu,OFFLINE_THRESHOLD2,OFFLINE_COUNT);
	/* chassis motor detect */
	judge_one_offline(fps.chassis[0],&sys_err->chassis[0],OFFLINE_THRESHOLD2,OFFLINE_COUNT);
	judge_one_offline(fps.chassis[1],&sys_err->chassis[1],OFFLINE_THRESHOLD2,OFFLINE_COUNT);
	judge_one_offline(fps.chassis[2],&sys_err->chassis[2],OFFLINE_THRESHOLD2,OFFLINE_COUNT);
	judge_one_offline(fps.chassis[3],&sys_err->chassis[3],OFFLINE_THRESHOLD2,OFFLINE_COUNT);
	/* chassis_imu detect */
	judge_one_offline(fps.chas_imu,&sys_err->chas_imu,OFFLINE_THRESHOLD1,OFFLINE_COUNT);
	/* dbus detect */
	judge_one_offline(fps.dbus,&sys_err->dbus,60,OFFLINE_COUNT);
	/* gimbal_imu detect */
	judge_one_offline(fps.pitch_imu,&sys_err->pitch_imu,OFFLINE_THRESHOLD1,OFFLINE_COUNT);
	/* judge detect */
	judge_one_offline(fps.judge,&sys_err->judge,20,OFFLINE_COUNT);
	/* nuc detect */
	judge_one_offline(fps.nuc,&sys_err->nuc,40,OFFLINE_COUNT);
	/* yaw motor detect */
	judge_one_offline(fps.yaw,&sys_err->yaw,OFFLINE_THRESHOLD1,OFFLINE_COUNT);
	/* pitch motor detect */
	judge_one_offline(fps.pitch,&sys_err->pitch,OFFLINE_THRESHOLD1,OFFLINE_COUNT);
	/* trigger motor detect */
	judge_one_offline(fps.trans,&sys_err->trans,OFFLINE_THRESHOLD1,OFFLINE_COUNT);
	
	if(sys_err->trans.is_err || \
		 sys_err->pitch.is_err || \
	   sys_err->yaw.is_err || \
	   sys_err->nuc.is_err ||
	   sys_err->judge.is_err ||
	   sys_err->chassis[0].is_err || \
		 sys_err->chassis[1].is_err || \
		 sys_err->chassis[2].is_err || \
	   sys_err->chassis[3].is_err || \
	   sys_err->board_imu.is_err  || \
	   sys_err->pitch_imu.is_err || \
	   sys_err->chas_imu.is_err || \
	   sys_err->dbus.is_err)
		s_infantry.state = INFANTRY_ALARM;
	else
		s_infantry.state = INFANTRY_RUN_WELL;
}
/**
 * @brief this function is to judge the state of one device 
 * @param fps-- the fps value per second
 * @param *err -- the device state struct 
 * @param offline_threshold -- the expect fps value 
 * @param offline_count -- a delay value to sure the device is really offline
 * @return None
 * @attention None
 */
void judge_one_offline(int fps,\
											 s_err_t *err,\
											 int offline_threshold,\
											 uint8_t offline_count)
{
	if(fps < offline_threshold)
		err->err_cnt++;
	else
		err->err_cnt = 0;
	if(err->err_cnt > offline_count)
		err->is_err = 1;
	else
		err->is_err = 0;
}
/***************** some alarm fuction**************************/
void Sys_Running_Well(void)
{
	 static uint8_t count = 0;
	 static uint8_t step = 0;
	 count++;
	 step++;
	 switch(step)
	 {
		 case 0:
			 break;
		 case 1:
			 HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
		 break;
		 case 2:
			 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		 break;
		 case 3:
			 HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_RESET);
		 break;
		 case 4:
			 HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_RESET);
		 break;
		 case 5:
			 HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_RESET);
		 break;
		 case 6:
			 HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_RESET);
		 break;
		 case 7:
			 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		 break;
		 case 8:
			 HAL_GPIO_WritePin(LED_H_GPIO_Port, LED_H_Pin, GPIO_PIN_RESET);
		 break;
		 case 9:
			 HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
		 break;
		 case 10:
			 HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		 break;
		 case 11:
			 HAL_GPIO_TogglePin(LED_C_GPIO_Port, LED_C_Pin);
		 break;
		 case 12:
			 HAL_GPIO_TogglePin(LED_D_GPIO_Port, LED_D_Pin);
		 break;
		 case 13:
			 HAL_GPIO_TogglePin(LED_E_GPIO_Port, LED_E_Pin);
		 break;
		 case 14:
			 HAL_GPIO_TogglePin(LED_F_GPIO_Port, LED_F_Pin);
		 break;
		 case 15:
			 HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
		 break;
		 case 16:
			 HAL_GPIO_TogglePin(LED_H_GPIO_Port, LED_H_Pin);
				step = 0;
		 break;
		 default :
			 break;
	 }	 
	 count > 9 ? (HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin), count = 0) : NULL;
}

void Sys_initializing(void)
{
	static uint16_t time_count = 0;
	time_count ++;
	if( time_count > 2 )
	{
		HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		HAL_GPIO_TogglePin(LED_C_GPIO_Port, LED_C_Pin);
		HAL_GPIO_TogglePin(LED_D_GPIO_Port, LED_D_Pin);
		HAL_GPIO_TogglePin(LED_E_GPIO_Port, LED_E_Pin);
		HAL_GPIO_TogglePin(LED_F_GPIO_Port, LED_F_Pin);
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
		HAL_GPIO_TogglePin(LED_H_GPIO_Port, LED_H_Pin);
		time_count = 0;
	}
}

/* 按照优先级顺序显示LED，当有离线发生，10个LED熄灭， 按照离线顺序显示 若离线，对应的LED亮起 对应关系	如下*/
/* ********朝着电源输入端为LED_A
*  LED_A  				      chassis[0]
*  LED_B  			        chassis[1]
*	 LED_C							  chassis[2]
*	 LED_D								chassis[3]
*  LED_E								yaw
*  LED_F                pitch
*	 LED_G								trigger
*	 LED_H								chassis_imu
*	 LED_GREEN						judge_system
*	 LED_RED							nuc
*/
void SysModule_Offline(void)
{
	if(s_sys_err.nuc.is_err)
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.judge.is_err)
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.chassis[0].is_err)
		HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.chassis[1].is_err)
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.chassis[2].is_err)
		HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.chassis[3].is_err)
		HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.yaw.is_err)
		HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.pitch.is_err)
		HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.trans.is_err)
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	
	if(s_sys_err.chas_imu.is_err)
		HAL_GPIO_WritePin(LED_H_GPIO_Port, LED_H_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H_GPIO_Port, LED_H_Pin, GPIO_PIN_SET);
}

/*响四声-遥控器离线
*响三声-陀螺仪离线
*响两声-电机离线
*响一声-裁判系统或NUC离线
*/
void Sys_Alarming(void)
{
	if(s_infantry.state == INFANTRY_ALARM)
	{
		if(s_sys_err.dbus.is_err)
			Di_Di_Di_Di();
		else
		{
			if(s_sys_err.pitch_imu.is_err || s_sys_err.board_imu.is_err || s_sys_err.chas_imu.is_err)
				Di_Di_Di();
			else if(s_sys_err.yaw.is_err || s_sys_err.pitch.is_err || s_sys_err.trans.is_err || \
								s_sys_err.chassis[0].is_err || s_sys_err.chassis[1].is_err || s_sys_err.chassis[2].is_err || s_sys_err.chassis[3].is_err)
				Di_Di();
			else if(s_sys_err.nuc.is_err || s_sys_err.judge.is_err)
				Di();
		}
	}
	else if(s_infantry.state == INFANTRY_RUN_WELL)
	{
		/*if(vision_info.CenterZ.f)
			Biu_biubiu();
		else*///not define now
			DoNot_Di();
	}
	else
		DoNot_Di();

}
