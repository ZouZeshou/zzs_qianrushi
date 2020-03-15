#ifndef PARAMETER_H
#define PARAMETER_H
#include "config.h"
/************ constant *******************************/
#define ANGLE_ENCODE            	22.7527778f//角度换算成码盘值（8191/360）
#define ENCODE_ANGLE            	0.0439506f //码盘值换算成角度（360/8191）
#define RPM_DPS                 	6 //转每分换算成度每秒（1/60*360）
#define ANGLE_RAD               	0.0174532f //角度换算成弧度（3.1415926/180）
#define RAD_ANGLE               	57.29578f//弧度换算成角度
/************ chassis *******************************/
#define MOVE_WSCONST  						8000//正常前后速度增幅
#define MOVE_ADCONST  						4000//正常左右速度增幅
#define E_WSCONST 	  						3000//慢速前进时速度增幅
#define CHANNEL_CONST             12.12f//底盘速度遥控器通道常数（660*12.12=8000）
#define CHASSIS_MAX_SPEED         8000//底盘单轮最大速度
#define SCALE_V_W                 0.35f//小陀螺模式下平移速度占旋转速度的最大占比
/************ gimbal *************************************/
#define CHANNEL_YAW_CONST         -0.048f//yaw遥控器通道常数
#define CHANNEL_PITCH_CONST       -0.012f//pitch遥控器通道常数
#define MOUSE_YAW_CONST           -0.8f//yaw鼠标常数
#define MOUSE_PITCH_CONST         -0.2f//pitch鼠标常数
#define TRANS_COMP_CONST 					0.09f//加在yaw输出量的拨盘输出量系数
/************ shoot ***************************************/
#define TRAVEL                    29487.6f//拨盘的单格行程（8191*36/10）
/************ vision ************************************/
#if  ROBOT_ID == 1
#define FRAMEDIFF_TO_SPD    			3.0f//帧差转化成速度的系数
#define FANWHEEL_CENTER_X   			250.0f//击打能量机关x中心
#define FANWHEEL_CENTER_Y					242.0f//击打能量机关y中心
#define ARMOR_OFFSET_X      			800.0f//击打机器人x中心
#define ARMOR_OFFSET_Y      			280.0f//击打机器人y中心
#define FAN_COMP_COEF       			0.01f//能量机关pitch补偿系数
#define ROBOT_PIT_COEF      			1.0f//装甲板pitch补偿系数
#define ROBOT_YAW_COEF      			0.4f//识别装甲时速度补偿系数
#define FAN_SHOOT_ERR       			1.0f//能量机关最小发弹误差
#define ROBOT_SHOOT_YAW_ERR 			2.0f//机器人x方向最小发弹误差
#define ROBOT_SHOOT_PIT_ERR 			1.5f//机器人y方向最小发弹误差
#define ROBOT_SHOOT_SPD_ERR 			150.0f//机器人速度最小发弹误差
#elif ROBOT_ID == 2

#elif ROBOT_ID == 3

#endif
#endif
