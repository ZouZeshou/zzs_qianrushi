#include "gimbal.h"
#include "STMGood.h"
#include "drv_keyboard.h"
#include "drv_can.h"
#include "usr_math.h"
#include "shoot.h"
#include "vision.h"
#include "drv_nuc_interface.h"
/***********some const value***********/

/***********definition of struct and enum*****/
uint8_t g_gimbal_move_mode = G_MANUAL;
uint8_t g_gimbal_info_src = G_GYRO;
pid_t s_yaw_pos_pid={0};
pid_t s_yaw_spd_pid={0};
pid_t s_pitch_pos_pid={0};
pid_t s_pitch_spd_pid={0};
s_motor_data_t s_yaw_motor = {YAW_ID,0};
s_motor_data_t s_pitch_motor = {PIT_ID,0};
/*************debug variable**************/
int g_yaw_pid_debug = 0;//debug mode or not
int g_pitch_pid_debug = 0;
/**
 * @brief initialize the parameter of gimbal
 * @param None
 * @return None
 * @attention None
 */
void gimbal_param_init(void)
{
	#if   ROBOT_ID == 1
	pid_struct_init(&s_yaw_pos_pid,400,10,25,0,0);//60 0 0
	pid_struct_init(&s_yaw_spd_pid,28000,20000,100,3.0f,0);//100 3 0
	pid_struct_init(&s_pitch_pos_pid,400,10,40,0,0);//60 0 0
	pid_struct_init(&s_pitch_spd_pid,28000,20000,100,3.0f,0);//100 3.0 0
	#elif ROBOT_ID == 2

	#elif ROBOT_ID == 3

	#endif
	s_yaw_motor.target_ang = 0.0f;
	s_yaw_motor.reduction_ratio = 1.0f;
	s_pitch_motor.reduction_ratio = 1.0f;
}
/**
 * @brief reset the pid parameter when you debug
 * @param None
 * @return None
 * @attention this function is for debug
 */
void gimbal_pid_param_reset(void)
{
	if(g_yaw_pid_debug)
	{
		pid_struct_init(&s_yaw_pos_pid,400,10,P,I,D);
		pid_struct_init(&s_yaw_spd_pid,28000,20000,p,i,d);
	}
	if(g_pitch_pid_debug)
	{
		pid_struct_init(&s_pitch_pos_pid,400,10,P,I,D);
		pid_struct_init(&s_pitch_spd_pid,28000,20000,p,i,d);
	}
}

/**
 * @brief switch the mode of shoot
 * @param None
 * @return None
 * @attention None
 */
void switch_gimbal_mode(uint8_t *gimbal_mode,\
												uint8_t vision_state,\
												s_motor_data_t *pitch,\
												s_motor_data_t *yaw)
{
	/********* switch mode from key***************/
	if(GIMBAL_LOCK_SW)
	{
		if(*gimbal_mode==G_LOCK)
			*gimbal_mode = G_MANUAL;
		else
			*gimbal_mode = G_LOCK;
	}
	
  if(vision_state == V_CATCH &&\
		g_vision_mode != V_NOT_USE &&\
		(!MANUAL_ATTACK) &&\
		(*gimbal_mode != G_LOCK))
	{
		*gimbal_mode = G_AUTO;	
	}
	if(*gimbal_mode == G_AUTO)
	{
		gimbal_param_init();
		yaw->target_ang = yaw->gyro_angle;
		pitch->target_pos = pitch->tol_pos;
		if(pitch->tol_pos < pitch->min_pos)
		{
			pitch->target_pos = pitch->min_pos;
			*gimbal_mode = G_MANUAL;	
		}
		else 	if(pitch->tol_pos > pitch->max_pos)
		{	
			pitch->target_pos = pitch->max_pos;
			*gimbal_mode = G_MANUAL;			
		}
	}		
}
/**
 * @brief gimbal control
 * @param None
 * @return None
 * @attention None
 */
void gimbal_ctrl(void)
{
	static uint8_t last_mode;
	switch(g_gimbal_move_mode)
	{
		case G_MANUAL:
		{
			get_gimbal_target_pos(&s_yaw_motor,&s_pitch_motor);
			if(g_gimbal_info_src == G_ENCODER)
			{
				calculate_serial_pid_current(&s_yaw_motor,&s_yaw_pos_pid,&s_yaw_spd_pid);
				calculate_serial_pid_current(&s_pitch_motor,&s_pitch_pos_pid,&s_pitch_spd_pid);
			}
			else if(g_gimbal_info_src == G_GYRO)
			{
				calculate_serial_pid_current_by_gyro(&s_yaw_motor,&s_yaw_pos_pid,&s_yaw_spd_pid);
				calculate_serial_pid_current_by_gyro(&s_pitch_motor,&s_pitch_pos_pid,&s_pitch_spd_pid);
			}
			break;
		}
		case G_AUTO:
		{
			get_target_by_vision(&s_vision_info);
			s_yaw_motor.out_current = (int16_t)pid_calculate(&s_yaw_vision_spd_pid,\
																											s_yaw_motor.gyro_speed,\
																											s_vision_info.adjustX);
			s_pitch_motor.out_current = (int16_t)pid_calculate(&s_pitch_vision_spd_pid,\
																												s_pitch_motor.gyro_speed,\
																												s_vision_info.adjustY);
			break;
		}
		case G_LOCK:
		{
			if(last_mode!=G_LOCK)
			{
				s_yaw_motor.target_pos = s_yaw_motor.tol_pos;
			  s_yaw_motor.target_ang = s_yaw_motor.gyro_angle;
				s_pitch_motor.target_pos = s_pitch_motor.tol_pos;
			  s_pitch_motor.target_pos = float_constrain(s_pitch_motor.target_pos,\
																								 s_pitch_motor.min_pos,\
																								 s_pitch_motor.max_pos);
			}
			calculate_serial_pid_current(&s_yaw_motor,&s_yaw_pos_pid,&s_yaw_spd_pid);
			calculate_serial_pid_current(&s_pitch_motor,&s_pitch_pos_pid,&s_pitch_spd_pid);
			break;
		}
		default:
			break;
	}
	last_mode = g_gimbal_move_mode;
}
/**
 * @brief get the target position of yaw motor
 * @param None
 * @return None
 * @attention the target is encoder value
 */
void get_gimbal_target_pos(s_motor_data_t *s_yaw,s_motor_data_t *s_pitch)
{
	s_yaw->target_pos += (((RC_Ctl.rc.ch2 - 1024)*CHANNEL_YAW_CONST) +\
																	 RC_Ctl.mouse.x * MOUSE_YAW_CONST);
	s_yaw->target_ang += (((RC_Ctl.rc.ch2 - 1024)*CHANNEL_YAW_CONST) +\
																	 RC_Ctl.mouse.x * MOUSE_YAW_CONST)*ENCODE_ANGLE;
	s_pitch->target_pos -= (((RC_Ctl.rc.ch3 - 1024)*CHANNEL_PITCH_CONST) +\
																		 RC_Ctl.mouse.y * MOUSE_PITCH_CONST);
	s_pitch->target_pos = float_constrain(s_pitch->target_pos,s_pitch->min_pos,s_pitch->max_pos);
}
/**
 * @brief calculate the current of motor which using 
 *  serial pid to control
 * @param *s_motor -- the motor data struct
 * @param *s_pos_pid -- postion pid struct
 * @param *s_spd_pid -- speed pid struct
 * @return None
 * @attention None
 */
void calculate_serial_pid_current_by_gyro(s_motor_data_t *s_motor,\
																					pid_t *s_pos_pid,\
																					pid_t *s_spd_pid)
{
	if(s_motor->id == YAW_ID)
	{
		s_motor->target_speed = pid_calculate(s_pos_pid,\
																				s_motor->gyro_angle,\
																				s_motor->target_ang);
	}
	else if(s_motor->id == PIT_ID)
	{
		s_motor->target_speed = pid_calculate(s_pos_pid,\
																				s_motor->tol_pos*ENCODE_ANGLE/s_motor->reduction_ratio,\
																				s_motor->target_pos*ENCODE_ANGLE/s_motor->reduction_ratio);
	}
	s_motor->out_current = (int16_t)pid_calculate(s_spd_pid,\
																			 s_motor->gyro_speed,\
																			 s_motor->target_speed);
}
