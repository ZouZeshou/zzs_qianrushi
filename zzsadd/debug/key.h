#ifndef KEY_H
#define KEY_H
#include "config.h"
/*************** key funtion define************/
#define LONG_PRESS_TIME  		 1000//按键长按的触发时间
#define FORWARD_NORMAL    	 (RC_Ctl.key.bit.W)//前进
#define FORWARD_VERY_SLOWLY  (RC_Ctl.key.bit.E)//缓慢前进
#define BACK        				 (RC_Ctl.key.bit.S)//后退
#define LEFT       					 (RC_Ctl.key.bit.A)//左移
#define RIGHT      					 (RC_Ctl.key.bit.D)//右移
#define OPEN_MAGAZINE				 (RC_Ctl.key.bit.R)//打开弹仓
#define CLOSE_MAGAZINE       (RC_Ctl.key.bit.CTRL && RC_Ctl.key.bit.R)//关闭弹仓
#define	WANT_USE_CAP	   		 (RC_Ctl.key.bit.SHIFT)//使用电容放电
#define MANUAL_ATTACK				 (RC_Ctl.mouse.r)//取消自瞄
#define MANUAL_SHOOT				 (RC_Ctl.mouse.l_press)//开火
#define ROBOT_ATTACK         (RC_Ctl.key.bit.Z)//视觉目标为机器人
#define SMALL_FAN_ATTACK		 (RC_Ctl.key.bit.X)//视觉目标为小能量机关
#define BIG_FAN_ATTACK       (RC_Ctl.key.bit.C)//视觉目标为大能量机关
#define GIMBAL_LOCK_SW       (s_keymouse.F_state == KEY_PRESS_ONCE)//进入/退出锁云台模式
#define CHASSIS__GYRO_SW     (s_keymouse.V_state == KEY_PRESS_ONCE)//进入/退出小陀螺模式 

#endif
