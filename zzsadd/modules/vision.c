/******************************************************************************/
/** @file Vision.c
 *  @version 2.0
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
#include "drv_keyboard.h"
#include "shoot.h"
#include "gimbal.h"
#include "arm_math.h"
#include "monitor_task.h"
#define NO_YAW_COMP         1//取消yaw补偿
#define NO_PIT_COMP         1//取消pitch补偿
#define NO_FAN_PIT_COMP     1//取消大符pitch补偿
#define H1        					2169//能量机关距地面高度  		
#define H2						      850 //击打点距地面高度			
#define H3          				350	//步兵枪口距着地点高度
#define FAN_DIS             7300.0f//能量机关距击打点水平距离
#if  ROBOT_ID == 1
#define FANWHEEL_CENTER_X   250.0f
#define FANWHEEL_CENTER_Y		242.0f
#define ARMOR_OFFSET_X      800.0f
#define ARMOR_OFFSET_Y      280.0f
#define FAN_COMP_COEF       0.01f//能量机关pitch补偿系数
#define ROBOT_PIT_COEF      1.0f//装甲板pitch补偿系数
#define ROBOT_YAW_COEF      0.4f//识别装甲时速度补偿系数
#define FAN_SHOOT_ERR       1.0f//最小发弹误差
#define ROBOT_SHOOT_YAW_ERR 2.0f//最小发弹误差
#define ROBOT_SHOOT_PIT_ERR 1.5f//最小发弹误差
#define ROBOT_SHOOT_SPD_ERR 150.0f//最小发弹误差
#elif ROBOT_ID == 2

#elif ROBOT_ID == 3

#endif
uint8_t g_vision_mode = V_ROBOT;
uint8_t g_vision_state = V_ABNORMAL;
pid_t  s_yaw_track_pid={0};
pid_t  s_pit_track_pid={0};
pid_t  s_yaw_fanwheel_pid={0};
pid_t  s_pitch_fanwheel_pid={0};
pid_t s_yaw_vision_spd_pid={0};
pid_t s_pitch_vision_spd_pid={0};

kalman1_state s_target_state_kal={0};
kalman1_state s_framediff_x_kal={0};
kalman1_state s_tar_abs_spd_kal={0};
kalman1_state s_fanwheel_centerX_kal={0};
kalman1_state s_fanwheel_centerY_kal={0};

int yaw_track_pid_debug = 0;
int pitch_track_pid_debug = 0;
int yaw_fanwheel_pid_debug = 0;
int pitch_fanwheel_pid_debug = 0;
int g_yaw_vision_debug = 0;
int g_pitch_vision_debug = 0;
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
	memset(&s_vision_info,0,sizeof(s_vision_t));
	#if ROBOT_ID == 1
	/* pid params */
	pid_struct_init(&s_yaw_track_pid, 250.0f, 0.0f, 110.0f, 0.00f, 0.0f);
	pid_struct_init(&s_pit_track_pid, 200.0f, 0.0f, 24.0f, 0.00f, 0.0f);//42
	pid_struct_init(&s_yaw_fanwheel_pid, 250.0f, 0.0f, 2.20f, 0.00f, 0.0f);//105
	pid_struct_init(&s_pitch_fanwheel_pid, 200.0f, 0.0f, 1.8f, 0.00f, 0.0f);	
	pid_struct_init(&s_yaw_vision_spd_pid,28000,20000,100.0f,2.0f,0.0f);
	pid_struct_init(&s_pitch_vision_spd_pid,28000,20000,80.0f,2.0f,0.0f);
	#elif ROBOT_ID == 2
	/* pid params */
	
	#elif ROBOT_ID == 3
	
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
	if(g_yaw_vision_debug)
	{
		pid_struct_init(&s_yaw_vision_spd_pid,28000,20000,P,I,D);
	}
	if(g_pitch_vision_debug)
	{
		pid_struct_init(&s_pitch_vision_spd_pid,28000,20000,P,I,D);
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
	if(BIG_FAN_ATTACK)
		*e_vision_mode = V_BIG_FAN;
	else if(SMALL_FAN_ATTACK)
		*e_vision_mode = V_SMALL_FAN;
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
	if(s_sys_err.nuc.is_err)
	{
		*vision_state = V_ABNORMAL;
		vision_param_init();
		lose_target_cnt = 0;
	}
	else
	{
		if((s_vision_info.is_find_target))
		{
			*vision_state = V_CATCH;
			lose_target_cnt = 0;
		}		
		else if(lose_target_cnt++ > 2)
		{
			  vision_param_init();
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
	if(g_vision_mode==V_BIG_FAN||g_vision_mode==V_SMALL_FAN)
	{//是否采用滤波后的值还有待测试
		vision->pit_comp = get_fan_pitch_comp(FAN_DIS,get_fan_y_target(),s_judge.max_spd);
		vision->pit_comp *= FAN_COMP_COEF;
		vision->Yaw_err = (FANWHEEL_CENTER_X - vision->CenterX.f)*ENCODE_ANGLE;
		if(NO_FAN_PIT_COMP)
			vision->Pitch_err = (FANWHEEL_CENTER_Y -  vision->CenterY.f)*ENCODE_ANGLE;  
		else
			vision->Pitch_err = (FANWHEEL_CENTER_Y -  vision->CenterY.f)*ENCODE_ANGLE + vision->pit_comp;
		vision->adjustX	= pid_calculate(&s_yaw_fanwheel_pid,vision->CenterX.f,FANWHEEL_CENTER_X);
		vision->adjustY	= pid_calculate(&s_pitch_fanwheel_pid,vision->CenterY.f,FANWHEEL_CENTER_Y);
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
		case V_SMALL_FAN:case V_BIG_FAN:
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
 * @brief calculate the error of yaw from speed of target 
 * @param None
 * @return None
 * @attention the speed direction must be the same with yaw motor 
 */
void get_yaw_err_from_vision(s_vision_t *vision)
{
	//yaw compensate coefficient
	vision->yaw_comp_coef = (-1.395f)*pow(10,-9)*(vision->CenterZ.f)*(vision->CenterZ.f) +\
									(1.103f)*pow(10,-5)*(vision->CenterZ.f) + 0.01039f;
	if(vision->CenterZ.f > 3600.0f)
		vision->yaw_comp_coef = 0.038f;
	vision->yaw_comp_coef = float_constrain(vision->yaw_comp_coef,0.020f,0.030f);//0.2-0.38
	vision->yaw_comp = vision->yaw_comp_coef * vision->target_abs_spd_kf;
	vision->yaw_comp *= ROBOT_YAW_COEF;
	if(NO_YAW_COMP)
		vision->Yaw_err =  (ARMOR_OFFSET_X - vision->CenterX.f ) * ENCODE_ANGLE;
	else
	  vision->Yaw_err =  (ARMOR_OFFSET_X - vision->CenterX.f ) * ENCODE_ANGLE + vision->yaw_comp;
}
/**
 * @brief calculate the error of pitch from vision data 
 * @param None
 * @return None
 * @attention 
 */
void get_pitch_err_from_vision(s_vision_t *vision)
{
	vision->pitch_initial_angle = get_robot_pitch_initial_angle();
	vision->pit_comp = get_robot_pitch_comp(vision->CenterZ.f,\
																		vision->pitch_initial_angle ,\
																		s_judge.max_spd);
	vision->pit_comp *= ROBOT_PIT_COEF;
  if(NO_PIT_COMP)
		vision->Pitch_err =  (ARMOR_OFFSET_Y - vision->CenterY.f)* ENCODE_ANGLE ;
	else
		vision->Pitch_err =  (ARMOR_OFFSET_Y - vision->CenterY.f)* ENCODE_ANGLE + vision->pit_comp;

			
}
/**
 * @brief get pitch compensation 
 * @param dist--离目标的直线距离
 * @param initial_angle--没加补偿前的角度
 * @param bullet_speed--子弹射速
 * @return 补偿角增量
 * @attention 
 */
float get_robot_pitch_comp(float dist, float initial_angle, float bullet_speed)
{
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    float GRAVITY = 9.7887f; //shenzhen 9.7887 
    y_temp = dist * sinf(initial_angle*ANGLE_RAD);
	  float y_final = y_temp;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++) 
	  {
        // 计算枪口抬升角度
        a = (float) atan2(y_temp, dist);
        // 计算实际落点
        float t = dist / (bullet_speed * cosf(a));
        y_actual = bullet_speed * sinf(a) * t - GRAVITY * t * t / 2;
        dy = y_final - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 10) 
				{
            break;
        }
    }
		float comp = a*RAD_ANGLE - initial_angle;
		if(comp < 0)
			comp = 0;
    return comp;
}
/**
 * @brief 得到pitch与水平面的夹角 
 * @attention 
 */
float get_robot_pitch_initial_angle(void)
{
	float angle_now =  (s_pitch_motor.tol_pos - s_pitch_motor.mid_pos)*ENCODE_ANGLE;
	float angle_initial = angle_now + (ARMOR_OFFSET_Y - s_vision_info.CenterY.f)*ENCODE_ANGLE;
	return angle_initial;
}
/**
 * @brief 得到目标高度值(能量机关)
 *能量机关中心高度 					2169  		H1
 *步兵激活点高度  						850 			H2
 *步兵枪口离地面高度 				350				H3
 *激活点到能量机关水平距离 		7300      dis
 *待打击叶片与水平面的夹角             theta
 */
float get_fan_y_target(void)
{
	float center_y = H1 - H2 - H3;
	float dy = 1400 * sinf(s_vision_info.fan_angle.f*ANGLE_RAD);
	float y_target = center_y + dy;
	return y_target;
}
/**
 * @brief 得到pitch补偿(能量机关)
 *待打击叶片与水平面的夹角             theta
 */
float get_fan_pitch_comp(float dist, float tvec_y, float bullet_spd)
{
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    float GRAVITY = 9.7887f; //shenzhen 9.7887  
    y_temp = tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++) 
	  {
        // 计算枪口抬升角度
        a = (float) atan2(y_temp, dist);
        // 计算实际落点
        float t, y = 0.0;
        t = dist / (bullet_spd * cos(a));
        y_actual = bullet_spd * sin(a) * t - GRAVITY * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 10) 
				{
            break;
        }
    }
    return y_temp - tvec_y;
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
			(fabs(vision->CenterX.f - FANWHEEL_CENTER_X) < FAN_SHOOT_ERR )&&\
			(fabs(vision->CenterY.f - FANWHEEL_CENTER_Y) < FAN_SHOOT_ERR))
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
	x_err = ROBOT_SHOOT_YAW_ERR - (vision->CenterZ.f-1000.0f)/1000.0f;
	x_err = float_constrain(x_err,ROBOT_SHOOT_YAW_ERR/3.0f,ROBOT_SHOOT_YAW_ERR);
	y_err = ROBOT_SHOOT_PIT_ERR - (vision->CenterZ.f-1000.0f)/2000.0f;
	y_err = float_constrain(y_err,ROBOT_SHOOT_PIT_ERR/3.0f,ROBOT_SHOOT_PIT_ERR);
	x_spd_err = ROBOT_SHOOT_SPD_ERR - (vision->CenterZ.f-1000.0f)/20.0f;
	x_spd_err = float_constrain(x_spd_err,ROBOT_SHOOT_SPD_ERR/3.0f,ROBOT_SHOOT_SPD_ERR);
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
/******************************** 保留 ，未使用 *************************************/
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
		vision->pit_comp = atan2f(falldowndist, (vision->CenterZ.f)/1000.0f);
		vision->pit_comp *= 57.29578f;
		//printf("pitch->pit_comp_angle = %f\r\n", pitch->pit_comp_angle);
			
	}
	return 1;
}