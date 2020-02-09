#include "chassis.h"
#include "STMGood.h"
#include "ramp.h"
#include "arm_math.h"
#include "math.h"
#include "usr_math.h"
#include "stdlib.h"
#include "drv_can.h"
#include "drv_judgesystem.h"
/***********definition of struct and enum*****/
uint8_t g_chassis_move_mode = C_LOCK;
uint8_t g_chassis_power_mode = C_NORMAL;
pid_t s_chassis_spd_pid[4]={0};
pid_t s_follow_pos_pid={0};
pid_t s_follow_spd_pid={0};
pid_t s_swing_pid = {0};
s_motor_data_t s_chassis_motor[4] = {0};
s_chassis_t s_chassis = {0};
ramp_t s_ramp_Vx = RAMP_DEFAULT_INIT;
ramp_t s_ramp_Vy = RAMP_DEFAULT_INIT;
ramp_t s_ramp_channelx = RAMP_SLOW_INIT;
ramp_t s_ramp_channely = RAMP_SLOW_INIT;
int chassis_wheel_pid_debug = 0;
int chassis_follow_pid_debug = 0;
int chassis_swing_pid_debug = 0;
/**
 * @brief initialize the parameter of chassis
 * @param None
 * @return None
 * @attention None
 */
void chassis_param_init(void)
{
	#if    ROBOT_ID == 1
	pid_struct_init(&s_chassis_spd_pid[0],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[1],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[2],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[3],10000,4000,6,0.1,0);
	pid_struct_init(&s_follow_pos_pid,1000,0,30.0f,0,0);//10
	pid_struct_init(&s_follow_spd_pid,8000,3000,6.0f,0,0);//20
	pid_struct_init(&s_swing_pid,7000,3000,4,0.01f,2);
	#elif  ROBOT_ID == 2
	pid_struct_init(&s_chassis_spd_pid[0],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[1],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[2],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[3],10000,4000,6,0.1,0);
	pid_struct_init(&s_follow_pos_pid,1000,0,30.0f,0,0);//10
	pid_struct_init(&s_follow_spd_pid,8000,3000,6.0f,0,0);//20
	pid_struct_init(&s_swing_pid,7000,3000,4,0.01f,2);
	#elif  ROBOT_ID == 3
	pid_struct_init(&s_chassis_spd_pid[0],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[1],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[2],10000,4000,6,0.1,0);
	pid_struct_init(&s_chassis_spd_pid[3],10000,4000,6,0.1,0);
	pid_struct_init(&s_follow_pos_pid,1000,0,30.0f,0,0);//10
	pid_struct_init(&s_follow_spd_pid,8000,3000,6.0f,0,0);//20
	pid_struct_init(&s_swing_pid,7000,3000,4,0.01f,2);
	#endif
	g_chassis_move_mode = C_FOLLOW;
	g_chassis_power_mode = C_NORMAL;
	s_chassis.power_ctrl.max_power = 40;/* unit:w */
}
/**
 * @brief reset the pid parameter when you debug
 * @param None
 * @return None
 * @attention this function is for debug
 */
void chassis_pid_param_reset(void)
{
	if(chassis_wheel_pid_debug)
	{
		pid_struct_init(&s_chassis_spd_pid[0],10000,4000,P,I,D);
		pid_struct_init(&s_chassis_spd_pid[1],10000,4000,P,I,D);
		pid_struct_init(&s_chassis_spd_pid[2],10000,4000,P,I,D);
		pid_struct_init(&s_chassis_spd_pid[3],10000,4000,P,I,D);
	}
	if(chassis_follow_pid_debug)
	{
		pid_struct_init(&s_follow_pos_pid,1600,0,P,I,D);
		pid_struct_init(&s_follow_spd_pid,8000,3000,p,i,d);
	}
	if(chassis_swing_pid_debug)
	{
		pid_struct_init(&s_swing_pid,7000,3000,P,I,D);
	}
}

/**
 * @brief switch the mode of chassis
 * @param None
 * @return None
 * @attention None
 */
void switch_chassis_mode(uint8_t *chassis_mode,\
												 s_keymouse_t s_key,\
												 s_chassis_t *s_chas,\
												 s_motor_data_t s_yaw)
{
	/************some variable*********/
	static int chas_mode_sw = 0;
		/***** deal the angle to -180 to +180 
	       the angle = chassis - gimbal(CCW is positive)*****/ 
	s_chas->angle_diff = (YAWMID - s_yaw.back_position)*ENCODE_ANGLE;
	s_chas->angle_diff = loop_float_constrain(s_chas->angle_diff, -180.0f, 180.0f);
	/****** switch the mode from keyboard**********/ 
	if(s_key.V_state == KEY_PRESS_ONCE)
	{
		chas_mode_sw = 1;
	}
	if(chas_mode_sw)
	{
		if(*chassis_mode == C_FOLLOW)
		{
			*chassis_mode = C_ROTATE;
			chas_mode_sw = 0;
		}
		else if(*chassis_mode == C_ROTATE)
		{
			if(s_chas->angle_diff >= -60 && s_chas->angle_diff <= 0)
			{
				*chassis_mode = C_FOLLOW;
				chas_mode_sw = 0;
			}
		}
	}
}
/**
 * @brief get the chassis speed from gimbal view
 * @param s_rc-- remote data struct
 * @param *s_chas -- the chassis data struct 
 * @return None
 * @attention the spd is from gimbal view 
 */
void get_spd_from_keyboard(RC_Ctl_t s_rc,s_chassis_t *s_chas)
{
	int16_t Vx_buffer = 0;
	int16_t Vy_buffer = 0;
	if(abs(s_rc.rc.ch1-1024) < 10 && abs(s_rc.rc.ch0-1024) < 10)
	{
		ramp_init(&s_ramp_channelx);
		ramp_init(&s_ramp_channely);
		s_chas->gim_Vx = 0;
		s_chas->gim_Vy = 0;
	}
	else
	{
		s_chas->gim_Vx = (s_rc.rc.ch1-1024)*CHANNEL_X_CONST * ramp_cal(&s_ramp_channelx);
		s_chas->gim_Vy = (s_rc.rc.ch0-1024)*CHANNEL_Y_CONST * ramp_cal(&s_ramp_channely);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       s_chas->gim_Vy = (s_rc.rc.ch0-1024)*CHANNEL_Y_CONST * ramp_cal(&s_ramp_channely);
	}
	
	/************** choose the move speed constant**************/
	if(FORWARD_SLOWLY)
	{
		Vx_buffer = Q_WSCONST;
		Vy_buffer = Q_ADCONST;
	}
	else if(FORWARD_VERY_SLOWLY)
	{
		Vx_buffer = E_WSCONST;
		Vy_buffer = E_ADCONST;		
	}
	else
	{
		/*if(chassis.can_use_cap == 1){
			Vx_buffer = MOVE_WSCONST;
			Vy_buffer = MOVE_ADCONST;
		}
		else{
			
		}*/
		Vx_buffer = MOVE_WSCONST;
		Vy_buffer = MOVE_ADCONST;
	}
	/************** calculate Vx from keyboard QWES*************/
	if(FORWARD_NORMAL || FORWARD_SLOWLY || FORWARD_VERY_SLOWLY)
	{
		if((FAST_SPD ))
		{
			s_chas->gim_Vx += Vx_buffer;
			ramp_cal(&s_ramp_Vx);
		}
		else
		{
			s_chas->gim_Vx += Vx_buffer * ramp_cal(&s_ramp_Vx);
		}
	}
	else if(BACK)
	{
		if((FAST_SPD))
		{
			s_chas->gim_Vx -= Vx_buffer;
			ramp_cal(&s_ramp_Vx);
		}
		else
		{
			s_chas->gim_Vx -= Vx_buffer * ramp_cal(&s_ramp_Vx);
		}
	}
	else
	{
		ramp_init(&s_ramp_Vx);
	}
	/************** calculate Vy from keyboard ADZX*************/
	if(RIGHT)
	{
			s_chas->gim_Vy += Vy_buffer;
	}
	else if(LEFT)
	{ 
			s_chas->gim_Vy -= Vy_buffer;
	}
	else if(s_rc.key.bit.Z)
	{
			s_chas->gim_Vy -= Vy_buffer;
			s_chas->gim_Vy /= 7;
	}
	else if(LEFT_SLOWLY)
	{
			s_chas->gim_Vy += Vy_buffer;
			s_chas->gim_Vy /= 7;
	}
	else
	{
		ramp_init(&s_ramp_Vx);
	}
	/*if((Remote.key.val & KEY_SHIFT) && (Remote.key.val & KEY_W)){
		usecap = 1;
	}else{
		usecap = 0;
	}*/
	s_chas->gim_Vx=int16_constrain(s_chas->gim_Vx,-8000,8000);
	s_chas->gim_Vy=int16_constrain(s_chas->gim_Vy,-8000,8000);
}
/**
 * @brief transform_chassis_spd
 * @param None
 * @return None
 * @attention None
 */
void transform_chassis_spd(s_chassis_t *s_chas)
{
	/*********** get the chassis speed from speed on gimbal view ***/ 
	switch(g_chassis_move_mode)
	{
		case C_FOLLOW:
		{
			s_chas->Vx = s_chas->gim_Vx * cosf(s_chas->angle_diff * DEGREE_RAD) -\
									 s_chas->gim_Vy * sinf(s_chas->angle_diff * DEGREE_RAD);
			//s_chas->Vy = s_chas->gim_Vx * sinf(s_chas->angle_diff * DEGREE_RAD) +\
									 s_chas->gim_Vy * cosf(s_chas->angle_diff * DEGREE_RAD);
			pid_calculate(&s_follow_pos_pid,0,-s_chas->angle_diff);
			s_chas->W = pid_calculate(&s_follow_spd_pid,s_chas->gyro_spd,s_follow_pos_pid.out);
			break;
		}
		case C_ROTATE:
		{
			switch(s_chas->power_ctrl.max_power)
			{
				case 50:
				{
					s_chas->W = 4000;
					s_chas->angle_diff += 2;
					break;
				}
				case 70:
				{
					s_chas->W = 6000;
					s_chas->angle_diff += 6.5f;
					break;
				}
				case 100:
				{
					s_chas->W = 8000;
					s_chas->angle_diff += 10.0f;
					break;
				}
				default:
					break;
			}
			s_chas->Vx = s_chas->gim_Vx * cosf(s_chas->angle_diff * DEGREE_RAD) -\
									 s_chas->gim_Vy * sinf(s_chas->angle_diff * DEGREE_RAD);
			s_chas->Vy = s_chas->gim_Vx * sinf(s_chas->angle_diff * DEGREE_RAD) +\
									 s_chas->gim_Vy * cosf(s_chas->angle_diff * DEGREE_RAD);
			int chassis_V = sqrt(s_chas->Vx * s_chas->Vx + s_chas->Vy * s_chas->Vy);
			if(chassis_V > (fabs(s_chas->W * SCALE_V_W)))
			{
				s_chas->Vx *= (fabs(s_chas->W * SCALE_V_W)) / chassis_V;
				s_chas->Vy *= (fabs(s_chas->W * SCALE_V_W)) / chassis_V;
			}
			break;
		}
		case C_SWING:
		{
			static int direction = 0;
			switch(direction)
			{
				case 0:
				{
					s_chas->W = pid_calculate(&s_swing_pid,s_chas->angle_diff,30)*ANGLE_ENCODE;
				  if(s_chas->angle_diff > 20)
					{
						direction = 1;
					}
					break;
				}
				case 1:
				{
					s_chas->W = pid_calculate(&s_swing_pid,s_chas->angle_diff,-30)*ANGLE_ENCODE;
				  if(s_chas->angle_diff < -20)
					{
						direction = 0;
					}
					break;
				}
			}
			s_chas->Vx = s_chas->gim_Vx * cos(s_chas->angle_diff * DEGREE_RAD) -\
									 s_chas->gim_Vy * sin(s_chas->angle_diff * DEGREE_RAD);
			s_chas->Vy = s_chas->gim_Vx * sin(s_chas->angle_diff * DEGREE_RAD) +\
									 s_chas->gim_Vy * cos(s_chas->angle_diff * DEGREE_RAD);
			break;
		}
		case C_LOCK:
		{
			s_chas->Vx = 0;
			s_chas->Vy = 0;
			s_chas->W = 0;
			break;
		}
		default:
			break;
	}
}
/**
 * @brief get the chassis speed
 * @param *s_chas--the chassis data struct
 * @param *s_motor_0-- the motor data struct
 * @return None
 * @attention input Vx,Vy and W,output the speed of four chassis motor
 */
void get_chassis_spd(s_chassis_t *s_chas,\
										 s_motor_data_t *s_motor_0,\
										 s_motor_data_t *s_motor_1,\
										 s_motor_data_t *s_motor_2,\
										 s_motor_data_t *s_motor_3)
{
	float Buffer[4], Param, MaxSpeed;
  uint8_t index;
  Buffer[0] = s_chas->Vx + s_chas->Vy - s_chas->W;
  Buffer[1] = -s_chas->Vx + s_chas->Vy - s_chas->W;
  Buffer[2] = s_chas->Vx - s_chas->Vy - s_chas->W;
  Buffer[3] = -s_chas->Vx - s_chas->Vy - s_chas->W; 
  //оч╥Ы
  for(index = 0, MaxSpeed = 0; index < 4; index++)
  {
     if((Buffer[index] > 0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
     {
        MaxSpeed = (Buffer[index] > 0 ? Buffer[index] : -Buffer[index]);
     }
  }
  if(s_chas->power_ctrl.max_spd < MaxSpeed)
  {
      Param = (float)CHASSIS_MAX_SPEED / MaxSpeed;
      s_motor_0->target_speed = Buffer[0] * Param;
      s_motor_1->target_speed = Buffer[1] * Param;
      s_motor_2->target_speed = Buffer[2] * Param;
      s_motor_3->target_speed = Buffer[3] * Param; 
  }
  else
  {
      s_motor_0->target_speed = Buffer[0];
      s_motor_1->target_speed = Buffer[1];
      s_motor_2->target_speed = Buffer[2];
      s_motor_3->target_speed = Buffer[3];
  }
}

/**
 * @brief limit the current
 * @param None
 * @return None
 * @attention None
 */
void limit_chassis_current(float max_value,int16_t *cur_1,int16_t *cur_2,int16_t *cur_3,int16_t *cur_4)
{
	static float cur_sum = 0;
	*cur_1 = int16_constrain(*cur_1,-max_value/3,max_value/3);
	*cur_2 = int16_constrain(*cur_2,-max_value/3,max_value/3);
	*cur_3 = int16_constrain(*cur_3,-max_value/3,max_value/3);
	*cur_4 = int16_constrain(*cur_4,-max_value/3,max_value/3);
	cur_sum = abs(*cur_1)+abs(*cur_2)+abs(*cur_3)+abs(*cur_4);
	if(cur_sum > max_value)
	{
		*cur_1 *= max_value/cur_sum;
		*cur_2 *= max_value/cur_sum;
		*cur_3 *= max_value/cur_sum;
		*cur_4 *= max_value/cur_sum;
	}
}
/**
 * @brief calculate the power parameter from judge and backspeed
 * @param None
 * @return None
 * @attention None
 */
void calculate_power_param(s_power_control_t *s_power,int16_t spd1,int16_t spd2,int16_t spd3,int16_t spd4)
{
	static int16_t spd_sum = 0;
	spd_sum = abs(spd1) + abs(spd2) + abs(spd3) + abs(spd4);
	if(g_chassis_power_mode==C_NORMAL)
	{
		s_power->max_power = 50;/* unit:w */
//		s_power->power_spd_scale = 50.0f;
//		s_power->max_spd = 2000 + s_power->max_power * s_power->power_spd_scale;
//		s_power->max_spd = int16_constrain(s_power->max_spd,2000,10000);
//		s_power->min_cur_sum = 1000 + s_power->max_power * 25;
//		s_power->max_cur_sum = 4000 + s_power->max_power * 200;
//		if(g_chassis_move_mode==C_ROTATE)
//		{
//			s_power->spd_cur_scale = 2.8f;
//		}
//		else if(g_chassis_move_mode==C_FOLLOW)
//		{
//			s_power->spd_cur_scale = 3.3f;
//		}
//		s_power->current_sum = s_power->max_cur_sum - s_power->spd_cur_scale * spd_sum/4.0f ;
//		s_power->current_sum = float_constrain(s_power->current_sum,s_power->min_cur_sum,s_power->max_cur_sum);
//		if(Judge_PowerHeatData.chassis_power_buffer < 60)
//		{
//			s_power->current_sum = s_power->min_cur_sum + \
//														 Judge_PowerHeatData.chassis_power_buffer * 200;
//		}
		s_power->max_spd = 8000;
		s_power->current_sum = 35000;
	}
	else if(g_chassis_power_mode==C_USE_CAP)
	{
		
	}
}
