#ifndef CONFIG_H
#define CONFIG_H
/***************** for robot *****************/
#define ROBOT_ID                        1//机器人id
#define NO_JUDGE_SYSTEM                 1//无裁判系统
/******************* chassis *****************/
#define NO_POWER_LIMIT                  1//是否解除功率限制
/****************** onboard imu *************/
#define USE_MAHONY_METHOD               0//是否使用mahony算法解算
#define USE_LPF_ACCELARATION            1//加速度数据是否经过低通滤波
#define USE_IST8310_DATA                1//是否使用地磁计数据
#define GET_OFFSET_TIME                 300//初始化时采集偏置的时间
#define DEFAULT_IMU_TEMP                50//加热电阻的目标温度
/***************** adjust pid ************/
/*说明：各按键对应功能如下：（调节单环时使用“PID”，双环时外环“PID”,内环“pid”）
‘Q’= p+1  ‘W’= i+0.1 ‘E’= d+1    ‘U’= P+1  ‘I’= I+0.1 ‘O’= D+1
‘A’= p-1  ‘S’= i-0.1 ‘D’= d-1    ‘J’= P-1  ‘K’= I-0.1 ‘L’= D-1
‘R’= p=i=d=0                         ‘P’ = P=I=D=0
*/
#define CHASSIS_WHEEL_PID_DEBUG         0//调节底盘电机速度环pid
#define CHASSIS_FOLLOW_PID_DEBUG        0//调节底盘跟随pid
#define CHASSIS_SWING_PID_DEBUG         0//调节底盘扭腰pid
#define YAW_PID_DEBUG                   0//调节yaw pid
#define PITCH_PID_DEBUG                 0//调节pitch pid
#define TRANS_PID_DEBUG                 0//调节拨盘电机 pid
#define FRIC_PID_DEBUG                  0//调节摩擦轮 pid
#define YAW_ROBOT_PID_DEBUG             0//调节击打机器人yaw 外环pid
#define PITCH_ROBOT_PID_DEBUG           0//调节击打机器人pitch 外环pid
#define YAW_FAN_PID_DEBUG               0//调节击打能量机关yaw 外环pid
#define PITCH_FAN_PID_DEBUG             0//调节击打能量机关pitch 外环pid
#define YAW_VISION_SPD_DEBUG            0//调节视觉击打yaw 内环pid
#define PITCH_VISION_SPD_DEBUG          0//调节视觉击打pitch 外环pid
/******************** vision **************/
#define NO_YAW_COMP         						1//取消yaw补偿
#define NO_PIT_COMP         						1//取消pitch补偿
#define NO_FAN_PIT_COMP     						1//取消大符pitch补偿
#define H1        											2169//能量机关距地面高度  		
#define H2						      						850 //击打点距地面高度			
#define H3          										350	//步兵枪口距着地点高度
#define FAN_DIS             						7300.0f//能量机关距击打点水平距离
/******************** show message *************/
#define USE_PLOT                        0//是否使用匿名科创上位机画图
#define SHOW_MONITOR                    0//显示模块异常情况
#define SHOW_DBUS                       0//显示遥控器通道及鼠标值
#define SHOW_CHASSIS                    0//显示底盘相关信息
#define SHOW_SHOOT                      0//显示发射机构相关信息
#define SHOW_GIMBAL                     0//显示云台相关信息
#define SHOW_JUDGE                      0//显示裁判系统相关信息
#define SHOW_VISION                     0//显示视觉相关信息
#endif
