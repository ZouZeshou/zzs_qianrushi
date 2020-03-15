#include "shoot.h"
#include "drv_keyboard.h"
#include "drv_dbus.h"
#include "STMGood.h"
#include "gimbal.h"
#include "drv_can.h"
#include "usr_math.h"
#include "vision.h"
#include "drv_judgesystem.h"
#include "drv_io.h"
#include "drv_timer.h"
#include "drv_flash.h"
#include "arm_math.h"
/***********definition of struct and enum*****/
uint8_t g_shoot_mode = S_MANUAL;
pid_t s_trans_pos_pid={0};
pid_t s_trans_spd_pid={0};
pid_t s_fric_l_spd_pid = {0};
pid_t s_fric_r_spd_pid = {0};
s_motor_data_t s_trans_motor = {TRANS_ID,0};
s_motor_data_t s_fric_l_motor = {0};
s_motor_data_t s_fric_r_motor = {0};
/**
 * @brief initialize the parameter of shoot
 * @param None
 * @return None
 * @attention None
 */
void shoot_param_init(void)
{
	#if   ROBOT_ID == 1
	pid_struct_init(&s_trans_pos_pid,1250,100,20,0,0);
	pid_struct_init(&s_trans_spd_pid,10000,3000,30,0,0);
	pid_struct_init(&s_fric_l_spd_pid,15000,5000,15,0,0);
	pid_struct_init(&s_fric_r_spd_pid,15000,5000,15,0,0);
	#elif ROBOT_ID == 2
	pid_struct_init(&s_trans_pos_pid,1250,100,20,0,0);
	pid_struct_init(&s_trans_spd_pid,10000,3000,30,0,0);
	pid_struct_init(&s_fric_l_spd_pid,15000,5000,15,0,0);
	pid_struct_init(&s_fric_r_spd_pid,15000,5000,15,0,0);
	#elif ROBOT_ID == 3
	pid_struct_init(&s_trans_pos_pid,1250,100,20,0,0);
	pid_struct_init(&s_trans_spd_pid,10000,3000,30,0,0);
	pid_struct_init(&s_fric_l_spd_pid,15000,5000,15,0,0);
	pid_struct_init(&s_fric_r_spd_pid,15000,5000,15,0,0);
	#endif
	s_trans_motor.reduction_ratio = 36;
}
/**
 * @brief reset the pid parameter when you debug
 * @param None
 * @return None
 * @attention this function is for debug
 */
void shoot_pid_param_reset(void)
{
	if(FRIC_PID_DEBUG)
	{
		pid_struct_init(&s_fric_l_spd_pid,15000,5000,P,I,D);
		pid_struct_init(&s_fric_r_spd_pid,15000,5000,P,I,D);
	}
	if(TRANS_PID_DEBUG)
	{
		pid_struct_init(&s_trans_pos_pid,1250,200,P,I,D);
		pid_struct_init(&s_trans_spd_pid,10000,3000,p,i,d);
	}
}
/**
 * @brief switch the mode of shoot
 * @param None
 * @return None
 * @attention None
 */
void switch_shoot_mode(uint8_t *shoot_mode)
{
	static uint8_t last_mode;
	if(g_gimbal_move_mode  == G_AUTO)
		*shoot_mode = S_AUTO;
	else if((last_mode==G_AUTO)&&(g_gimbal_move_mode!=G_AUTO))
		*shoot_mode = S_MANUAL;
	last_mode = g_gimbal_move_mode;
	if(CLOSE_MAGAZINE)
		close_bullet_magazine();
	else if(OPEN_MAGAZINE)
		open_bullet_magazine();
}
/**
 * @brief shoot control
 * @param None
 * @return None
 * @attention None
 */
void shoot_ctrl(void)
{
	switch(g_shoot_mode)
	{
		case S_MANUAL:
		{
			get_fric_spd_from_realspd(s_judge.max_spd,&s_fric_l_motor,&s_fric_r_motor);
			if(MANUAL_SHOOT)
				shoot_by_heat(&s_judge.real_heat,s_judge.heat_reduce,s_judge.max_heat);
			break;
		}
		case S_AUTO:
		{
			vision_shoot_ctrl(&s_vision_info);
			break;
		}
		case S_CLEAR_BULLETS:
		{
			s_fric_l_motor.target_speed = 4500;
			s_fric_r_motor.target_speed = -s_fric_l_motor.target_speed;
			shoot_by_heat(&s_judge.real_heat,s_judge.heat_reduce,s_judge.max_heat);
			break;
		}
		case S_JUST_SHOOT:
		{
			s_fric_l_motor.target_speed = 7000;
			s_fric_r_motor.target_speed = -s_fric_l_motor.target_speed;
			shoot_by_heat(&s_judge.real_heat,s_judge.heat_reduce,s_judge.max_heat);
			break;
		}
		default:
			break;
	}
}

/**
 * @brief deal the dicrete encode to continue data
 * @param None
 * @return None
 * @attention None
 */
void continue_motor_pos(s_motor_data_t *s_motor)
{
	if(s_motor->is_pos_ready == 1)
	{
		if(s_motor->back_position - s_motor->back_pos_last > 4096)
		{
			s_motor->circle_num--;
		}
		else if(s_motor->back_position - s_motor->back_pos_last < -4096)
		{
			s_motor->circle_num++;
		}
	}
	else
	{
		#if ROBOT_ID == 1
		if(s_motor->id == YAW_ID)
		{
			if(s_motor->back_position>=0&&s_motor->back_position<=7000)
			{	
				s_motor->max_pos = 5000;
				s_motor->mid_pos = 2700;
				s_motor->min_pos = 0;
				s_motor->target_pos = s_motor->mid_pos;
			}
			else if(s_motor->back_position>7000)
			{
				s_motor->max_pos = 5000+8191;
				s_motor->mid_pos = 2700+8191;
				s_motor->min_pos = 0+8191;
				s_motor->target_pos = s_motor->mid_pos;
			}
		}
		else if(s_motor->id == PIT_ID)
		{
				s_motor->max_pos = 6850;
				s_motor->mid_pos = 6120;
				s_motor->min_pos = 5600;
				s_motor->target_pos = s_motor->mid_pos;
		}
		#elif ROBOT_ID == 2
		if(s_motor->id == YAW_ID)
		{
			if(s_motor->back_position>=0&&s_motor->back_position<=7000)
			{	
				s_motor->max_pos = 5000;
				s_motor->mid_pos = 2700;
				s_motor->min_pos = 0;
				s_motor->target_pos = s_motor->mid_pos;
			}
			else if(s_motor->back_position>7000)
			{
				s_motor->max_pos = 5000+8191;
				s_motor->mid_pos = 2700+8191;
				s_motor->min_pos = 0+8191;
				s_motor->target_pos = s_motor->mid_pos;
			}
		}
		else if(s_motor->id == PIT_ID)
		{
				s_motor->max_pos = 6850;
				s_motor->mid_pos = 6120;
				s_motor->min_pos = 5600;
				s_motor->target_pos = s_motor->mid_pos;
		}
		#elif ROBOT_ID == 3
		if(s_motor->id == YAW_ID)
		{
			if(s_motor->back_position>=0&&s_motor->back_position<=7000)
			{	
				s_motor->max_pos = 5000;
				s_motor->mid_pos = 2700;
				s_motor->min_pos = 0;
				s_motor->target_pos = s_motor->mid_pos;
			}
			else if(s_motor->back_position>7000)
			{
				s_motor->max_pos = 5000+8191;
				s_motor->mid_pos = 2700+8191;
				s_motor->min_pos = 0+8191;
				s_motor->target_pos = s_motor->mid_pos;
			}
		}
		else if(s_motor->id == PIT_ID)
		{
				s_motor->max_pos = 6850;
				s_motor->mid_pos = 6120;
				s_motor->min_pos = 5600;
				s_motor->target_pos = s_motor->mid_pos;
		}
		#endif
		else if(s_motor->id == TRANS_ID)
		{
			s_motor->target_pos = s_motor->back_position ;
		}
		s_motor->is_pos_ready = 1;
	}
	s_motor->back_pos_last =  s_motor->back_position;
	s_motor->tol_pos = s_motor->back_position + s_motor->circle_num * 8191;
}

/**
 * @brief calculate the current of motor which using 
 *  single pid to control
 * @param *s_motor -- the motor data struct
 * @param *s_spd_pid -- speed pid struct
 * @return None
 * @attention None
 */
void calculate_single_pid_current(s_motor_data_t *s_motor,pid_t *s_spd_pid)
{
	s_motor->out_current = pid_calculate(s_spd_pid,\
																			 s_motor->back_speed,\
																			 s_motor->target_speed);
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
void calculate_serial_pid_current(s_motor_data_t *s_motor,\
																	pid_t *s_pos_pid,\
																	pid_t *s_spd_pid)
{
	s_motor->target_speed = pid_calculate(s_pos_pid,\
																				s_motor->tol_pos*ENCODE_ANGLE/s_motor->reduction_ratio,\
																				s_motor->target_pos*ENCODE_ANGLE/s_motor->reduction_ratio);
	if(s_motor->id == YAW_ID||s_motor->id == PIT_ID)
	{
		s_motor->out_current = (int16_t)pid_calculate(s_spd_pid,\
																			 s_motor->gyro_speed,\
																			 s_motor->target_speed);
	}
	else
	{
		s_motor->out_current = pid_calculate(s_spd_pid,\
																			 s_motor->back_speed*RPM_DPS/s_motor->reduction_ratio,\
																			 s_motor->target_speed);
	}
	
}
/**
 * @brief calculate the friction speed from real speed
 * @param None
 * @return None
 * @attention None
 */
void get_fric_spd_from_realspd(float real_spd,\
															 s_motor_data_t * s_fric_l,\
															 s_motor_data_t * s_fric_r)
{
	/* matlab fit function */
	s_fric_l->target_speed = (int )(-3.0f*real_spd*real_spd+360.0f*real_spd);
	s_fric_l->target_speed = int16_constrain(s_fric_l->target_speed,4000,8000);
	s_fric_r->target_speed = -s_fric_l->target_speed;
}
/** 
	* @param *energy 当前热量
	* @param heat_reduce 每秒冷却
	* @param max_heat 最大热量上限
	* @return shoot success--1 fail--0
*/
int shoot_by_heat(uint16_t *energy,uint8_t heat_reduce,uint8_t max_heat)
{
	static int energyuse = 0;
	int xenergy = (int)*energy;
	energyuse = max_heat - xenergy;
	int energyadd = 10;
	if((energyuse - 10) < energyadd)/*this condition need test*/  
	{
		s_trans_motor.frequency = 0;
		return 0;
	}
	else
	{
		if(g_vision_mode == V_BIG_FAN||g_vision_mode == V_SMALL_FAN)
		{
			s_trans_motor.frequency = 3;
		}
		else 
		{
			/* 自瞄按左键 高爆发 */
			if((!MANUAL_ATTACK) && MANUAL_SHOOT)
			{
				if(energyuse < 100)
					s_trans_motor.frequency = energyuse/energyadd;
				else
					s_trans_motor.frequency = 20;
			}
			else
				s_trans_motor.frequency = energyuse/energyadd;
		}
		s_trans_motor.frequency = int_constrain(s_trans_motor.frequency,0,20);
		int isshot = fire(5);//5 : the running freq of this functiong(Fire)
		if(isshot) 
			*energy += energyadd;
		return isshot;
		
	}
}
/**
 * @brief update the target of trans motor by frequency
 * @param isshoot--shoot or not
 * @param time_diff-- the running time interval of this function
 * @return 1--shoot success  0--shoot fail
 * @attention None
 */
int fire(uint8_t time_diff)
{
	static int count = 0;
	if((fabs(s_fric_l_spd_pid.err) < 1000.0f)&&\
  	 (fabs(s_fric_r_spd_pid.err) < 1000.0f))
	{
		count++;
	}
	else
		count = 0;
	if((count*time_diff > (int)(1000.0f/s_trans_motor.frequency))&&\
		(s_trans_motor.frequency != 0))
	{
		int is_shoot_over = shoot_once();
		count = 0;
		if(is_shoot_over) 
			return 1;
		return 0;
	}
	return 0;
}
/**
 * @brief shoot once if error is small
 * @return shoot success--1 fail--0
 * @attention None
 */
int shoot_once(void)
{
	if(fabs(s_trans_motor.target_pos - s_trans_motor.tol_pos ) < TRAVEL)
	{
		s_trans_motor.target_pos += TRAVEL;
		return 1;
	}
	return 0;
}
