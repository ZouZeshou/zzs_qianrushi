#include "debug_printf_task.h"
#include "infantry.h"
#include "shoot.h"
#include "chassis.h"
#include "gimbal.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_io.h"
#include "drv_imu.h"
#include "drv_dbus.h"
#include "monitor_task.h"
#include "stdint.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "drv_judgesystem.h"
#include "ANO_DT.h"
#include "drv_nuc_interface.h"
#include "drv_flash.h"
#include "vision.h"
#include "global.h"
void StartTask07(void const * argument)
{
  for(;;)
  {
	  send_data_to_nuc(20);
		printf_debug_data(4);
    osDelay(50);
  }
}
/**
 * @brief printf the data you need for debug
 * @param None
 * @return None
 * @attention None
 */
void printf_debug_data(uint8_t time_gap)
{
	static uint8_t count;
	if(count++ > time_gap)
	{
		count = 0;
		printf("/****************printf_task work well************/\r\n");
		if(USE_PLOT)
			ANO_DT_Data_Exchange();
		show_fps_data();
		print_debus();
		printf_judge();
		printf_chassis();
		printf_shoot();
		printf_gimbal();
	  print_vision();
	}
}
/**
 * @brief printf the fps data
 * @param None
 * @return None
 * @attention None
 */
void show_fps_data(void)
{
	if(SHOW_MONITOR)
	{
		printf("dbus iserr=%d\r\n", s_sys_err.dbus.is_err);
		printf("LF iserr=%d\r\n", s_sys_err.chassis[0].is_err);
		printf("RF iserr=%d\r\n", s_sys_err.chassis[1].is_err);
		printf("LB iserr=%d\r\n", s_sys_err.chassis[2].is_err);
		printf("RB iserr=%d\r\n", s_sys_err.chassis[3].is_err);
		printf("yaw iserr=%d\r\n", s_sys_err.yaw.is_err);
		printf("pitch iserr=%d\r\n", s_sys_err.pitch.is_err);
		printf("trans iserr=%d\r\n", s_sys_err.trans.is_err);
		printf("pit imu iserr=%d\r\n", s_sys_err.pitch_imu.is_err);
		printf("board imu iserr=%d\r\n", s_sys_err.board_imu.is_err);
		printf("chas imu iserr=%d\r\n", s_sys_err.chas_imu.is_err);
		printf("judgesystem iserr=%d\r\n", s_sys_err.judge.is_err);
		printf("nuc iserr=%d\r\n", s_sys_err.nuc.is_err);
	}
}
/**
 * @brief printf the dbus data
 * @param None
 * @return None
 * @attention None
 */
void print_debus(void)
{
	if(SHOW_DBUS)
	{
		printf("ch0 %d\r\n",RC_Ctl.rc.ch0);
		printf("ch1 %d\r\n",RC_Ctl.rc.ch1);
		printf("ch2 %d\r\n",RC_Ctl.rc.ch2);
		printf("ch3 %d\r\n",RC_Ctl.rc.ch3);
		printf("s1 %d,UP1,DOWN2,MID3\r\n",RC_Ctl.rc.s1);
		printf("s2 %d\r\n",RC_Ctl.rc.s2);
		printf("mouse_x %d\r\n",RC_Ctl.mouse.x);
		printf("mouse_y %d\r\n",RC_Ctl.mouse.y);
		printf("mouse_z %d\r\n",RC_Ctl.mouse.z);
		printf("mouse_l %d\r\n",RC_Ctl.mouse.l);
		printf("mouse_r %d\r\n",RC_Ctl.mouse.r);
	}
}
/**
 * @brief printf the chassis data
 * @param None
 * @return None
 * @attention None
 */
void printf_chassis(void)
{
	if(SHOW_CHASSIS)
	{
		printf("chassis_mode %d,F0,ROTATE1,SWING2,LOCK3\r\n",g_chassis_move_mode);
		printf("power_mode %d,NOCAP4,USECAP5\r\n",g_chassis_power_mode);
		printf("chassis anglediff %.2f \r\n",s_chassis.angle_diff);
		printf("chas_gyro ang %.2f spd %.2f\r\n",s_chassis.gyro_angle,s_chassis.gyro_spd);
		printf("gim Vx %d Vy %d W %d\r\n",s_chassis.gim_Vx,s_chassis.gim_Vy,s_chassis.gim_W);
		printf("chassis Vx %d Vy %d W %d\r\n",s_chassis.Vx,s_chassis.Vy,s_chassis.W);
	//	printf("sin90 %.2f cos90 %.2f\r\n",sinf(90*ANGLE_RAD),cosf(90*ANGLE_RAD));
	//	printf("sin90 %.2f cos90 %.2f\r\n",sinf(90.0f),cosf(90.0f));
	}
	if(CHASSIS_WHEEL_PID_DEBUG)
	{
		printf("LF pos %d spd %d\r\n",s_chassis_motor[0].back_position,s_chassis_motor[0].back_speed);
		printf("RF pos %d spd %d\r\n",s_chassis_motor[1].back_position,s_chassis_motor[1].back_speed);
		printf("LB pos %d spd %d\r\n",s_chassis_motor[2].back_position,s_chassis_motor[2].back_speed);
		printf("RB pos %d spd %d\r\n",s_chassis_motor[3].back_position,s_chassis_motor[3].back_speed);
		printf("LF err %.2f out %.2f\r\n",s_chassis_spd_pid[0].err,s_chassis_spd_pid[0].out);
		printf("RF err %.2f out %.2f\r\n",s_chassis_spd_pid[1].err,s_chassis_spd_pid[1].out);
		printf("LB err %.2f out %.2f\r\n",s_chassis_spd_pid[2].err,s_chassis_spd_pid[2].out);
		printf("RB err %.2f out %.2f\r\n",s_chassis_spd_pid[3].err,s_chassis_spd_pid[3].out);
	}
	if(CHASSIS_FOLLOW_PID_DEBUG)
	{
		printf("foll_pos err %.2f out %.2f\r\n",s_follow_pos_pid.err,s_follow_pos_pid.out);
		printf("foll_spd err %.2f out %.2f\r\n",s_follow_spd_pid.err,s_follow_spd_pid.out);
	}
	if(CHASSIS_SWING_PID_DEBUG)
		printf("swing err %.2f out %.2f\r\n",s_swing_pid.err,s_swing_pid.out);
}
/**
 * @brief printf the shoot data
 * @param None
 * @return None
 * @attention None
 */
void printf_shoot(void)
{
	if(SHOW_SHOOT)
	{
		printf("shoot_mode %d,M0,AUTO1,CLEAR3,FIRE4\r\n",g_shoot_mode);
		printf("trans cir %lld\r\n",s_trans_motor.circle_num);
		printf("trans tar %.1f spd %.1f\r\n",s_trans_motor.target_pos,s_trans_motor.target_speed);
		printf("trans pos %d tolpos %lld spd %d\r\n",s_trans_motor.back_position,\
																								 s_trans_motor.tol_pos,\
																								 s_trans_motor.back_speed);
	}
	if(TRANS_PID_DEBUG)
	{
		printf("trans_pos err %.2f out %.2f\r\n",s_trans_pos_pid.err,s_trans_pos_pid.out);
	  printf("trans_spd err %.2f out %.2f\r\n",s_trans_spd_pid.err,s_trans_spd_pid.out);
	}
	if(FRIC_PID_DEBUG)
	{
		printf("fric_l pos %d spd %d\r\n",s_fric_l_motor.back_position,s_fric_l_motor.back_speed);
		printf("fric_r pos %d spd %d\r\n",s_fric_r_motor.back_position,s_fric_r_motor.back_speed);
		printf("fric_l err %.2f out %.2f\r\n",s_fric_l_spd_pid.err,s_fric_l_spd_pid.out);
		printf("fric_r err %.2f out %.2f\r\n",s_fric_r_spd_pid.err,s_fric_r_spd_pid.out);
	}	
}
/**
 * @brief printf the gimbal data
 * @param None
 * @return None
 * @attention None
 */
void printf_gimbal(void)
{
	if(SHOW_GIMBAL)
	{
		printf("gimbal_mode %d ,M0 A1 LOCK2 \r\n",g_gimbal_move_mode);
		printf("gim_ctr_mode %d,E3 G4\r\n ",g_gimbal_info_src);
		printf("yaw pos %d targetangle %.2f spd %d tar %.2f\r\n",s_yaw_motor.back_position,\
																							 s_yaw_motor.target_ang,\
																							 s_yaw_motor.back_speed,\
																							 s_yaw_motor.target_pos);

		printf("pitch pos %d tolpos %lld spd %d tar %.2f\r\n",s_pitch_motor.back_position,\
																								 s_pitch_motor.tol_pos,\
																								 s_pitch_motor.back_speed,\
																								 s_pitch_motor.target_pos);
		printf("yaw_gyro ang %.2f spd %.2f\r\n",s_yaw_motor.gyro_angle,s_yaw_motor.gyro_speed);
		printf("yaw initial %.2f\r\n",INS_Angle[0] * 57.3f);
		printf("pitch_gyro ang %.2f spd %.2f\r\n",s_pitch_motor.gyro_angle,s_pitch_motor.gyro_speed);
	}
	if(YAW_PID_DEBUG)
	{
		printf("yawpos err %.2f out %.2f\r\n",s_yaw_pos_pid.err,s_yaw_pos_pid.out);
		printf("yawspd err %.2f out %.2f\r\n",s_yaw_spd_pid.err,s_yaw_spd_pid.out);
	}
	if(PITCH_PID_DEBUG)
	{
		printf("pitchpos err %.2f out %.2f\r\n",s_pitch_pos_pid.err,s_pitch_pos_pid.out);
		printf("pitchspd err %.2f out %.2f\r\n",s_pitch_spd_pid.err,s_pitch_spd_pid.out);
	}
}

void printf_judge(void)
{
	if(SHOW_JUDGE)
	{
		printf("heat_reduce = %d\r\n",s_judge.heat_reduce);
		printf("max_heat = %d\r\n",s_judge.max_heat);
		printf("max_power = %d\r\n",s_judge.max_power);
		printf("max_spd = %d\r\n",s_judge.max_spd);
		printf("power_buffer = %d\r\n",s_judge.power_buffer);
		printf("real_heat = %d\r\n",s_judge.real_heat);
		printf("real_id = %d\r\n",s_judge.real_id);
		printf("real_power = %d\r\n",s_judge.real_power);
	}
}
void print_vision(void)
{
	if(SHOW_VISION)
	{
		printf("visionmode %d ,NOUSE0,BIG1,SMALL2,ROBOT3\r\n",g_vision_mode);
		printf("visionstate %d,ABNORMAL3,LOSE4,CATCH5,\r\n",g_vision_state);
		printf("center x %.2f ,y %.2f z %.2f\r\n",\
						s_vision_info.CenterX.f,\
						s_vision_info.CenterY.f,\
						s_vision_info.CenterZ.f);
		printf("isbig = %d,fan_angle %.2f,validfps %d\r\n",\
						s_vision_info.is_big_armor,\
						s_vision_info.fan_angle.f,\
						s_vision_info.valid_fps);
		printf("adjust x %.2f y %.2f\r\n",s_vision_info.adjustX,s_vision_info.adjustY);
		printf("yaw_comp %.2f\r\n",s_vision_info.yaw_comp);
		printf("pitch_comp %.2f\r\n",s_vision_info.pit_comp);
		printf("target_spd %.2f\r\n",s_vision_info.target_abs_spd_kf);
		printf("yaw_comp_coefficient %.4f",s_vision_info.yaw_comp_coef);
		printf("yawcurrent %d pitchcurrent %d\r\n",s_yaw_motor.out_current,s_pitch_motor.out_current);
	}
	if(YAW_VISION_SPD_DEBUG)
		printf("yaw_vision_spd_pid err %.2f out %.2f\r\n",\
					s_yaw_vision_spd_pid.err,s_yaw_vision_spd_pid.out);
	if(PITCH_VISION_SPD_DEBUG)
		printf("pitch_vision_spd_pid err %.2f out %.2f\r\n",\
					s_pitch_vision_spd_pid.err,s_pitch_vision_spd_pid.out);
}
