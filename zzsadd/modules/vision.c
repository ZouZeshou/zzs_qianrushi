/******************************************************************************/
/** @file Vision.c
 *  @version 1.0
 *  @date 2020.1.27
 *  @brief the data are from the info-interface.c
 *  @author Echo
 *  @modify zzs
 */
#include "vision.h"
#include "string.h"
#include "STMgood.h" 
#include "usr_math.h"
#include "kalman_filter.h"
#include "ANO_DT.h"
#include "drv_judgesystem.h"
#include "drv_dbus.h"
#include "shoot.h"
#include "gimbal.h"
#include "arm_math.h"
#include "monitor_task.h"
#define MIN_ERR_SHOOT       15.0f
#if  ROBOT_ID == 1
#define FANWHEEL_CENTER_X   250.0f
#define FANWHEEL_CENTER_Y		242.0f
#define ARMOR_OFFSET_X      800.0f
#define ARMOR_OFFSET_Y      280.0f
#elif ROBOT_ID == 2
#define FANWHEEL_CENTER_X   250.0f
#define FANWHEEL_CENTER_Y		242.0f
#define ARMOR_OFFSET_X      800.0f
#define ARMOR_OFFSET_Y      280.0f
#elif ROBOT_ID == 3
#define FANWHEEL_CENTER_X   250.0f
#define FANWHEEL_CENTER_Y		242.0f
#define ARMOR_OFFSET_X      800.0f
#define ARMOR_OFFSET_Y      280.0f
#endif
uint8_t g_vision_mode = V_ROBOT;
uint8_t g_vision_state = V_ABNORMAL;
pid_t  s_yaw_track_pid;
pid_t  s_pit_track_pid;
pid_t  s_yaw_fanwheel_pid;
pid_t  s_pitch_fanwheel_pid;

kalman1_state s_target_state_kal;
kalman1_state s_framediff_x_kal;
kalman1_state s_tar_abs_spd_kal;
kalman1_state s_fanwheel_centerX_kal;
kalman1_state s_fanwheel_centerY_kal;

float yaw_comp_coef = 0;
float yaw_comp = 0;
int yaw_track_pid_debug = 0;
int pitch_track_pid_debug = 0;
int yaw_fanwheel_pid_debug = 0;
int pitch_fanwheel_pid_debug = 0;
/**
 * @brief initialize the parameter of vision
 * @param None
 * @return None
 * @attention None
 */
//落点偏左， 减小
//落点偏下， 增加
void vision_param_init(void)
{
	g_vision_mode = V_ROBOT;
	#if ROBOT_ID == 1
	/* pid params */
	pid_struct_init(&s_yaw_track_pid, 250.0f, 0.0f, 110.0f, 0.00f, 0.0f);
	pid_struct_init(&s_pit_track_pid, 200.0f, 0.0f, 24.0f, 0.00f, 0.0f);//42
	pid_struct_init(&s_yaw_fanwheel_pid, 250.0f, 0.0f, 220.0f, 0.00f, 0.0f);//105
	pid_struct_init(&s_pitch_fanwheel_pid, 200.0f, 0.0f, 1.8f, 0.00f, 0.0f);	

	#elif ROBOT_ID == 2
	/* pid params */
	pid_struct_init(&s_yaw_track_pid, 250.0f, 0.0f, 110.0f, 0.00f, 0.0f);
	pid_struct_init(&s_pit_track_pid, 200.0f, 0.0f, 24.0f, 0.00f, 0.0f);//42
	pid_struct_init(&s_yaw_fanwheel_pid, 250.0f, 0.0f, 220.0f, 0.00f, 0.0f);//105
	pid_struct_init(&s_pitch_fanwheel_pid, 200.0f, 0.0f, 1.8f, 0.00f, 0.0f);	
	#elif ROBOT_ID == 3
	/* pid params */
	pid_struct_init(&s_yaw_track_pid, 250.0f, 0.0f, 110.0f, 0.00f, 0.0f);
	pid_struct_init(&s_pit_track_pid, 200.0f, 0.0f, 24.0f, 0.00f, 0.0f);//42
	pid_struct_init(&s_yaw_fanwheel_pid, 250.0f, 0.0f, 220.0f, 0.00f, 0.0f);//105
	pid_struct_init(&s_pitch_fanwheel_pid, 200.0f, 0.0f, 1.8f, 0.00f, 0.0f);	
	#endif
	/* kalman1_init*/
	kalman1_init(&s_framediff_x_kal, 0, 100, 5, 50);
	kalman1_init(&s_tar_abs_spd_kal, 0, 500, 1, 50);
	kalman1_init(&s_fanwheel_centerX_kal, 0, 100, 10, 50);
	kalman1_init(&s_fanwheel_centerY_kal, 0, 100, 10, 50);
}

/**
 * @brief reset the parameter of vision
 * @param None
 * @return None
 * @attention None
 */
void vision_pid_pamar_reset(void)
{
	if(yaw_track_pid_debug)
	{
		pid_struct_init(&s_yaw_track_pid, 250.0f, 0.0f, P, I, D);
	}
	if(pitch_track_pid_debug)
	{
		pid_struct_init(&s_pit_track_pid, 200.0f, 0.0f, P, I, D);
	}
	if(yaw_fanwheel_pid_debug)
	{
		pid_struct_init(&s_yaw_fanwheel_pid, 250.0f, 0.0f, P, I, D);
	}
	if(pitch_fanwheel_pid_debug)
	{
		pid_struct_init(&s_pitch_fanwheel_pid, 200.0f, 0.0f, P, I, D);
	}
}
/**
 * @brief switch the mode of vision from key
 * @param None
 * @return None
 * @attention None
 */
void switch_vision_mode(uint8_t *e_vision_mode)
{
	if(FANWHEEL_ATTACK)
		*e_vision_mode = V_FANWHEEL;
	else if(GYRO_ATTACK)
		*e_vision_mode = V_GYRO;
	else if(ROBOT_ATTACK)
		*e_vision_mode = V_ROBOT;
}
/**
 * @brief vision state detect
 * @param None
 * @return None
 * @attention None
 */
void detect_vision_state(uint8_t *vision_state)
{
	static uint8_t lose_target_cnt = 0;
	static uint8_t catch_target_cnt = 0;
	if(s_sys_err.nuc.is_err)
	{
		*vision_state = V_ABNORMAL;
		memset(&s_vision_info,0,sizeof(s_vision_t));
		lose_target_cnt = 0;
		catch_target_cnt = 0;
	}
	else
	{
		if((s_vision_info.CenterX.f > 0.0f)&&(s_vision_info.valid_fps.d_16 > 30))
		{
			*vision_state = V_CATCH;
			lose_target_cnt = 0;
		}		
		else if(lose_target_cnt++ > 0)
		{
			  memset(&s_vision_info,0,sizeof(s_vision_t));
				*vision_state = V_LOSE;
			  lose_target_cnt = 0;
		}		
	}
}
/**
 * @brief get yaw and pitch final target  
 * @param input-- target_center and real_center and depth
 * @param output-- fric_spd  adjustX  adjustY
 * @return None
 * @attention None
 */
void get_target_by_vision(s_vision_t * vision)
{
	if(g_vision_mode == V_FANWHEEL)
	{//是否采用滤波后的值还有待测试
		vision->adjustX	= pid_calculate(&s_yaw_fanwheel_pid,\
																		vision->CenterX.f*0.01f,\
																		FANWHEEL_CENTER_X*0.01f);																	
		vision->adjustY	= pid_calculate(&s_pitch_fanwheel_pid,\
																		vision->CenterY.f,\
																		FANWHEEL_CENTER_Y);
	}
	else
	{
		get_yaw_err_from_vision(vision);
		get_pitch_err_from_vision(vision);
		vision->adjustX = pid_calculate(&s_yaw_track_pid,0,vision->Yaw_err);
		vision->adjustY =  pid_calculate(&s_pit_track_pid,0,vision->Pitch_err);
	}
}
/**
 * @brief shoot control of vision
 * @param None
 * @return None
 * @attention None
 */
void vision_shoot_ctrl(s_vision_t *vision)
{
	switch(g_vision_mode)
	{
		case V_FANWHEEL:
		{
			fanwheel_shoot_ctr(vision);
			break;
		}
		case V_GYRO:
		{
			gyro_shoot_ctr(vision);
			break;
		}
		case V_ROBOT:
		{
			robot_shoot_ctr(vision);
			break;
		}
		default:
			break;
	}
}

/**
 * @brief calculate the pitch angle add value
 * @param None
 * @return None
 * @attention None
 */
/* PITCH轴重力补偿 */
uint8_t get_pitch_compensate_angle(s_vision_t * vision,s_motor_data_t *pitch)
{
	if(vision->CenterZ.f != 0.0f)
	{
		/* 距离单位为m 故时间单位为s */
		float angle_theta = 0.0f;
		float fly_time = 0.0f;//单位 s
		float falldowndist = 0.0f;
		float gravity_acc = 9.80665;
		
		if(pitch->mid_pos < pitch->tol_pos)
			/* 得到pitch轴与水平的夹角theta，使用编码器计算 并转换弧度制 */
			angle_theta = (pitch->tol_pos - pitch->mid_pos) * 0.00076699f;//360.0f / 8192.0f * 3.1415926f / 180.0f;
		else
		{
			angle_theta = 0.0f;
			return 0;
		}
			
		//printf("angle_theta = %f\r\n", angle_theta);
		/* 计算飞行时间flytime */
		fly_time = vision->CenterZ.f / ((pitch->real_spd) * cosf(angle_theta));
		fly_time /= 1000.0f;
		//printf("fly_time = %f\r\n", fly_time);
		/* 计算子弹下落距离 */
		falldowndist = gravity_acc * fly_time * fly_time / 2.0f;
		//printf("falldowndist = %f\r\n", falldowndist);
		/* 计算补偿角 */
		vision->pit_comp_angle = atan2f(falldowndist, (vision->CenterZ.f)/1000.0f);
		vision->pit_comp_angle *= 57.29578f;
		//printf("pitch->pit_comp_angle = %f\r\n", pitch->pit_comp_angle);
			
	}
	return 1;
}
/**
 * @brief calculate the error of yaw from speed of target 
 * @param None
 * @return None
 * @attention the speed direction must be the same with yaw motor 
 */
void get_yaw_err_from_vision(s_vision_t *vision)
{
	//yaw compensate coefficient
	yaw_comp_coef = (-1.395f)*pow(10,-9)*(vision->CenterZ.f)*(vision->CenterZ.f) +\
									(1.103f)*pow(10,-5)*(vision->CenterZ.f) + 0.01039f;
	#if ROBOT_ID == 1
	yaw_comp_coef *= 0.4f;
	#elif ROBOT_ID == 2
	yaw_comp_coef *= 0.4f;
	#elif ROBOT_ID == 3
	yaw_comp_coef *= 0.4f;
	#endif
	if(vision->CenterZ.f > 3600.0f)
		yaw_comp_coef = 0.038f;
	yaw_comp_coef = float_constrain(yaw_comp_coef,0.020f,0.030f);//0.2-0.38
	yaw_comp = yaw_comp_coef * vision->target_abs_spd_kf;
	vision->Yaw_err =  (ARMOR_OFFSET_X - vision->CenterX.f ) * 0.01f +\
										 yaw_comp_coef * vision->target_abs_spd_kf;
	//vision->Yaw_err =  (ARMOR_OFFSET_X - vision->CenterX.f ) * 0.01f;
}
/**
 * @brief calculate the error of pitch from vision data 
 * @param None
 * @return None
 * @attention 
 */
void get_pitch_err_from_vision(s_vision_t *vision)
{
	get_pitch_compensate_angle(vision,&s_pitch_motor);	
//	if(vision->CenterX.f != 0.0f)
//			vision->Pitch_err =  (ARMOR_OFFSET_Y - vision->CenterY.f)* ENCODE_ANGLE +\
//														vision->pit_comp_angle;
//	else
			vision->Pitch_err =  (ARMOR_OFFSET_Y - vision->CenterY.f)* ENCODE_ANGLE ;
}
/**
 * @brief fanwheel shot control
 * @param None
 * @return None
 * @attention None
 */
void fanwheel_shoot_ctr(s_vision_t *vision)//110
{
		get_fric_spd_from_realspd(s_judge.max_spd,&s_fric_l_motor,&s_fric_r_motor);	
		if(vision->last_center_x != 0.0f && vision->CenterX.f != 0.0f&&\
			(fabs(vision->CenterX.f - FANWHEEL_CENTER_X) < MIN_ERR_SHOOT )&&\
			(fabs(vision->CenterY.f - FANWHEEL_CENTER_Y) < MIN_ERR_SHOOT))
		{//catch target and err is small
			shoot_by_heat(&s_judge.real_heat,s_judge.heat_reduce,s_judge.max_heat);
		}
}
/**
 * @brief robot shot control
 * @param None
 * @return None
 * @attention None
 */
void robot_shoot_ctr(s_vision_t *vision)
{
	static float x_err = 0;
	static float y_err = 0;
	static float x_spd_err = 0;
	get_fric_spd_from_realspd(s_judge.max_spd,\
														&s_fric_l_motor,\
														&s_fric_r_motor);
	x_err = 2.5f - (vision->CenterZ.f-1000.0f)/1000.0f;
	x_err = float_constrain(x_err,1.0f,2.5f);
	y_err = 2.2f - (vision->CenterZ.f-1000.0f)/2000.0f;
	y_err = float_constrain(y_err,1.5f,2.2f);
	x_spd_err = 130.0f - (vision->CenterZ.f-1000.0f)/20.0f;
	x_spd_err = float_constrain(x_spd_err,50.0f,130.0f);
	if((vision->CenterZ.f < 6000.0f) &&\
		(fabs(vision->Yaw_err) < x_err) &&\
		(fabs(vision->Pitch_err) < y_err)&&\
		(fabs(vision->target_abs_spd) < x_spd_err))
	{//catch the target
		shoot_by_heat(&s_judge.real_heat,s_judge.heat_reduce,s_judge.max_heat);
	}
}
/**
 * @brief gyro shot control
 * @param None
 * @return None
 * @attention None
 */
void gyro_shoot_ctr(s_vision_t *vision)
{
	get_fric_spd_from_realspd(s_judge.max_spd,&s_fric_l_motor,&s_fric_r_motor);
	shoot_by_heat(&s_judge.real_heat,s_judge.heat_reduce,s_judge.max_heat);
}
