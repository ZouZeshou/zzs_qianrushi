#ifndef _INFO_INTERFACE_H_
#define _INFO_INTERFACE_H_

#include "kalman_filter.h"
typedef union{
   float f;
	 unsigned char uc[4];
}float2uchar;

typedef union{
	int16_t d_16;
	unsigned char uc[4];
}int16uchar;

typedef struct 
{
	 float    Yaw_err;
	 float    yaw_comp;
	 float    yaw_comp_coef;
	 float 		Pitch_err;
	 float    pitch_initial_angle; 
	 float    pit_comp; 
	 float 		fanwheel_centerX_kf;
	 float 		fanwheel_centerY_kf;
	 float    centerX_kf;
	 float	  centerY_kf;
	 float    adjustX;
	 float    adjustY;
	
	 float2uchar CenterX;
	 float2uchar CenterY;
	 float2uchar CenterZ;
	 float2uchar fan_angle;
	 uint8_t     valid_fps;
	 uint8_t     fps;
	 uint8_t     is_big_armor;
	 uint8_t     is_find_target;

	 
	 float    last_center_x;
	 float    last_center_y;
	 float 		framediff_x;
	 float 		framediff_x_kf;//after kalman filter
	 float 		target_abs_spd;
	 float 		target_abs_spd_kf;
}s_vision_t;

extern s_vision_t s_vision_info;
extern kalman1_state s_framediff_x_kal;
extern kalman1_state s_tar_abs_spd_kal;
extern kalman1_state s_fanwheel_centerX_kal;
extern kalman1_state s_fanwheel_centerY_kal;

void send_data_to_nuc(int loopsforsend);
void send_data_to_mpu(uint8_t mode,int loopsforsend);
void vision_decode_handle(uint8_t *databuff);
void deal_nuc_data_by_bytes(uint8_t *buffaddr,uint8_t buff_size);
float Get_Data_Change_Part(float data, s_vision_t * nuc);
int16_t Get_Data_SeveralTimes_Ago(int16_t vision_fps, int16_t anglespdnow);
void get_target_spd(void);
#endif
