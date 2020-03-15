/*this file is some function to drive the infantry system*/
#include "infantry.h"
#include "shoot.h"
#include "chassis.h"
#include "gimbal.h"
#include "vision.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_io.h"
#include "drv_imu.h"
#include "drv_timer.h"
#include "drv_dbus.h"
#include "drv_judgesystem.h"
#include "iwdg.h"
#include "drv_buzzer.h"
#include "drv_flash.h"
#include "oled.h"
#include "drv_adc.h"
s_infantry_t s_infantry = {0};
s_judgesystem_t  s_judge = {0};
/**
 * @brief Enable the infantry system
 * @param None
 * @return None
 * @attention None
 */
void infantry_init(void)
{	
	s_infantry.state = INFANTRY_INITIALIZING;
	s_infantry.mode = NORMAL;
	/*************** imu and oled  adc *****************************************/
	mpu_device_init();
	oled_init();
	adc_init();
	/*************** parameter *****************************/
	cali_param_init();
	dubs_data_init();
	shoot_param_init();
	gimbal_param_init();
	chassis_param_init();
	vision_param_init();
	judgesystem_param_init();
	/***************** io and pwm timer *************************************/
	pwm_init();
	io_init();
	TIM_Enable();
	/*********** uart *****************************************/
	USART_DMA_Enable(&DBUS_HANDLE,&DBUS_DMA,dbus_buffer,DBUS_LENGTH);
	USART_DMA_Enable(&VISION_HANDLE,&VISION_DMA,vision_buffer,VISION_LENGTH);
	USART_Enable(&DEBUG_HANDLE,debug_buffer,DEBUG_LENGTH);
	/***************** can ***********************************/
	CAN_Enable(&hcan1);
	CAN_Enable(&hcan2);
	/******** state init *********************************/
	s_infantry.state = INFANTRY_RUN_WELL;
	play_music(1);
}
/**
 * @brief switch the mode from RC（这个函数处理的是遥控器模式的变化，键盘鼠标模式切换写在别的函数里）
 * @param None
 * @return None
 * @attention 这个函数必须放在键鼠模式切换前，因为键鼠优先级比遥控器高
 */
void rc_mode_switch(void)
{
	switch(RC_Ctl.rc.s1)
	{
		case UP:
		{//debug mode
			open_bullet_magazine();
			turn_on_laser();
			switch(RC_Ctl.rc.s2)
			{
				case UP:
				{//clear bullets
					g_chassis_move_mode =   C_FOLLOW;
					g_gimbal_move_mode  =   G_MANUAL;
					g_shoot_mode        =   S_CLEAR_BULLETS;
					g_vision_mode	      =   V_NOT_USE;					
					break;
				}
				case MID:
				{//static
					g_chassis_move_mode =   C_LOCK;
					g_gimbal_move_mode  =   G_LOCK;
					g_shoot_mode        =   S_MANUAL;
					g_vision_mode	      =   V_NOT_USE;					
					break;
				}
				case DOWN:
				{//add bullets
					g_chassis_move_mode =   C_FOLLOW;
					g_gimbal_move_mode  =   G_MANUAL;
					g_shoot_mode        =   S_MANUAL;
					g_vision_mode	      =   V_NOT_USE;
					break;
				}
				default:
					break;
			}
			break;
		}
		case MID:
		{//manual mode
			close_bullet_magazine();
			turn_on_laser();
			switch(RC_Ctl.rc.s2)
			{
				case UP:
				{//rotate
					g_chassis_move_mode =   C_ROTATE;
					g_gimbal_move_mode  =   G_MANUAL;
					g_shoot_mode        =   S_MANUAL;
					g_vision_mode	      =   V_NOT_USE;					
					break;
				}
				case MID:
				{//follow
					if((s_chassis.angle_diff>=-90&&s_chassis.angle_diff<=-20)||g_chassis_move_mode==C_LOCK)
						g_chassis_move_mode =   C_FOLLOW;
					g_gimbal_move_mode  =   G_MANUAL;
					g_shoot_mode        =   S_MANUAL; 
					g_vision_mode	      =   V_NOT_USE;					
					break;
				}
				case DOWN:
				{//lock
					g_chassis_move_mode =   C_LOCK;
					g_gimbal_move_mode  =   G_LOCK;
					g_shoot_mode        =   S_MANUAL;
					g_vision_mode	      =   V_NOT_USE;
					break;
				}
				default:
					break;
			}
			break;
		}
		case DOWN:
		{//vision mode
			//turn_off_laser();
			switch(RC_Ctl.rc.s2)
			{
				case UP:
				{//attack fanwheel
					g_chassis_move_mode =   C_FOLLOW;
					g_gimbal_move_mode  =   G_MANUAL;
					g_shoot_mode        =   S_MANUAL;
					g_vision_mode	      =   V_BIG_FAN;				
					break;
				}
				case MID:
				{//attack gyro
					g_chassis_move_mode =   C_FOLLOW;
					g_gimbal_move_mode  =   G_MANUAL;
					g_shoot_mode        =   S_MANUAL;
					g_vision_mode	      =   V_ROBOT;						
					break;
				}
				case DOWN:
				{//use keyboard to control robot
					break;
				}
				default:
					break;
			}
			break;
		}
		default:
			break;
	}
}
