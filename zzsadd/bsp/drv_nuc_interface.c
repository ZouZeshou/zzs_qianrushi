/******************************************************************************/
/** @file info-interface.c
 *  @version 1.0
 *  @date 2019.1.19
 *
 *  @brief info-interface
 *
 *  @author Echo
 *	@change zzs
 */
#include "drv_nuc_interface.h"
#include "CRC.h"
#include "math.h"
#include "drv_uart.h"
#include "ANO_DT.h"
#include "drv_judgesystem.h"
#include "gimbal.h"
#include "monitor_task.h"
#include "vision.h"
#include "global.h"
#include "drv_uart.h"
#include "usr_math.h"
s_vision_t s_vision_info;
uint8_t vision_data_buff[23];	
/**
 * @brief  send the data to nuc through the uart6
 * @param 	mode--attack fanwheel or robot(fanwheel 1 robot 2)
 * @param  loopsforsend-- the period to send message
 * @return  
 * @attention  
 */
void send_data_to_nuc(int loopsforsend)
{
	static int tic;
	tic++;
	if(tic >= loopsforsend)
	{
		tic = 0;
		uint8_t data[7];
		data[0] = 0xA0;
		data[1] = s_judge.real_id;
		data[2] = g_vision_mode;
		data[3] = 0;		
		Append_CRC8_Check_Sum(data,5);
		data[5] = '\r';                      
		data[6] = '\n';
//		for(int i = 0;i < 23;i++)
//		{
//			USART_Send_Char(&huart7,data[i]);
//		}
		HAL_UART_Transmit(&huart7,data,7,20);
	}
}
/**
 * @brief  send the data to nuc through the uart6
 * @param 	mode--attack fanwheel or robot(fanwheel 1 robot 2)
 * @param  loopsforsend-- the period to send message
 * @return  
 * @attention  
 */
void deal_nuc_data_by_bytes(uint8_t *buffaddr,uint8_t buff_size)
{
	static uint8_t header_ok;
	static uint8_t data_index;
	static uint8_t visionbuff[50];
	uint8_t receive_full_data = 0;
	printf("decode \r\n");
	for(int i = 0;i<buff_size;i++)
	{
		uint8_t data = buffaddr[i];
		if(data == 0xA5 )
		{
			printf("header ok \r\n");
			header_ok = 1;
			visionbuff[0] = data;
			data_index = 1;
		}
		else if((header_ok == 1) && (data_index < buff_size))
		{
			visionbuff[data_index++] = data;
			if(data_index == buff_size)
			{
				receive_full_data = 1;
				printf("decode success\r\n");
				vision_decode_handle(visionbuff);
			}
		}
		else
		{
				header_ok = 0;
		}
	}
}
/**
 * @brief  send the data to nuc through the uart6
 * @param 	mode--attack fanwheel or robot(fanwheel 1 robot 2)
 * @param  loopsforsend-- the period to send message
 * @return  
 * @attention  
 */
void send_data_to_mpu(uint8_t mode,int loopsforsend)
{
	static int tic;
	tic++;
	if(tic >= loopsforsend)
	{
		tic = 0;
		uint8_t tdata[23];
		tdata[0] = 0xA5;

    tdata[1] = 1;
    tdata[2] = 1;
    tdata[3] = 1;
    tdata[4] = 1;

    tdata[5] = 2;
    tdata[6] = 2;
    tdata[7] = 2;
    tdata[8] = 2;

    tdata[9] = 3;
    tdata[10] = 3;
    tdata[11] = 3;
    tdata[12] = 3;

    tdata[13] = 4;
    tdata[14] = 4;
    tdata[15] = 4;
    tdata[16] = 4;

    tdata[17] = 5;
    tdata[18] = 5;
    tdata[19] = 5;
    tdata[20] = 5;

    Append_CRC16_Check_Sum(tdata,23);
//		for(int i = 0;i < 23;i++)
//		{
//			USART_Send_Char(&huart6,tdata[i]);
//		}
//		HAL_UART_Transmit(&huart6,tdata,23,20);
	}
}
/**
 * @brief  receive the nuc data through the uart6
 * @param 	*databuff--the databuffer of nuc data	
 * @param 	*nuc--the databuffer of nuc data	
 * @return  None
 * @attention  call it ini the UART6 DMA receive interrupt
 */
void vision_decode_handle(uint8_t *databuff)
{
	if(databuff[0] == 0xA5 && Verify_CRC16_Check_Sum(databuff,23))
	{
		s_vision_info.CenterX.uc[0] = databuff[1];
		s_vision_info.CenterX.uc[1] = databuff[2];
		s_vision_info.CenterX.uc[2] = databuff[3];
		s_vision_info.CenterX.uc[3] = databuff[4];

		s_vision_info.CenterY.uc[0] = databuff[5];
		s_vision_info.CenterY.uc[1] = databuff[6];
		s_vision_info.CenterY.uc[2] = databuff[7];
		s_vision_info.CenterY.uc[3] = databuff[8];
		
		s_vision_info.CenterZ.uc[0] = databuff[9];
		s_vision_info.CenterZ.uc[1] = databuff[10];
		s_vision_info.CenterZ.uc[2] = databuff[11];
		s_vision_info.CenterZ.uc[3] = databuff[12];
		
		s_vision_info.fps = databuff[13];
		s_vision_info.valid_fps = databuff[14];
		s_vision_info.is_find_target = databuff[15];
		s_vision_info.is_big_armor = databuff[16];	
		
		s_vision_info.fan_angle.uc[0] = databuff[17];
		s_vision_info.fan_angle.uc[0] = databuff[18];
		s_vision_info.fan_angle.uc[0] = databuff[19];
		s_vision_info.fan_angle.uc[0] = databuff[20];
				
		if(g_vision_mode == V_BIG_FAN||g_vision_mode == V_SMALL_FAN)
		{
			s_vision_info.fanwheel_centerX_kf = kalman1_filter(&s_fanwheel_centerX_kal,s_vision_info.CenterX.f);
			s_vision_info.fanwheel_centerY_kf = kalman1_filter(&s_fanwheel_centerY_kal,s_vision_info.CenterY.f);
		}
		else//attack robot
		{
			if((s_vision_info.last_center_x != 0)&&s_vision_info.is_find_target&&\
				  fabs(s_vision_info.CenterX.f  - s_vision_info.last_center_x) < 150.0f)
			{//message is right and catch target
					get_target_spd();
			}
		}
		s_vision_info.last_center_x = s_vision_info.CenterX.f;
		s_vision_info.last_center_y = s_vision_info.CenterY.f;
		s_fps.nuc ++;
	}
}

/**
 * @brief  本函数是将can收到的信息进行处理，得到一定时间之前的值，将其作用到视觉的补偿量求解上去， 抵消视觉的滞后效应 
 * @param 	视觉的帧率
 * @return   一定时间以前的云台角速度
 * @author		Echo
 */
int16_t Get_Data_SeveralTimes_Ago(int16_t vision_fps, int16_t anglespdnow)
{
	static int16_t anglespd_collect[1000] = {0};
	static uint16_t Index = 0;
	static int16_t Spd_Result = 0;
	//HSV: 144：25  165 22   BGR: 189 : 20  210 - 225 :15
	//uint16_t Index_threshold = 15 + (200 - vision_fps)/5;
	uint16_t Index_threshold = 30;
	anglespd_collect[Index] = anglespdnow;
	if(Index >= Index_threshold)
	{
		Spd_Result = anglespd_collect[0];
		for(int i = 0; i < Index_threshold; i++)
		{
			anglespd_collect[i] = anglespd_collect[i+1];
		}
		return Spd_Result;
	}
	else
		Index++;	
	return 0;
}

/**
 * @brief  get speed of target
 * @attention  the positive direction of yaw is ccw
               so the positive speed of target is left
							 so when target move right,the speed is negative
 */
void get_target_spd(void)
{
	s_vision_info.framediff_x = FRAMEDIFF_TO_SPD * (s_vision_info.CenterX.f - s_vision_info.last_center_x);
	s_vision_info.framediff_x_kf = kalman1_filter(&s_framediff_x_kal,s_vision_info.framediff_x);
	if(!s_sys_err.board_imu.is_err)
			s_vision_info.target_abs_spd =  s_yaw_motor.anglespd_times_ago -\
																		  s_vision_info.framediff_x_kf;
	else
		s_vision_info.target_abs_spd = 0;
	s_vision_info.target_abs_spd_kf = kalman1_filter(&s_tar_abs_spd_kal,s_vision_info.target_abs_spd);
	s_vision_info.target_abs_spd == 0 ? s_vision_info.target_abs_spd_kf = 0 : NULL ;
	fabs(s_vision_info.target_abs_spd_kf) < 5 ? s_vision_info.target_abs_spd_kf = 0 : NULL;
	s_vision_info.target_abs_spd_kf = float_constrain(s_vision_info.target_abs_spd_kf,-200.0f,200.0f);
}

/* 本函数是对于帧差积分后会有一个积分定值出现的现象进行处理，
 * 将定值进行判断，然后减去，使得到的补偿量即保证变化趋势，
 * 又能稳定后保证在零值 */
//float Get_Data_Change_Part(float data, s_vision_t * nuc)
//{
//	static float old_Data[3];
//	static uint32_t count = 0;
//	static float compare_base = 0.0f;
//	static float last_centerX = 0.0f;
//	if(count == 0)
//	{
//		old_Data[0] = data;
//		count = 1;
//	}
//	else if(count == 1)
//	{
//		count = 2;
//		old_Data[1] = data;	
//	}
//	else if(count == 2)
//	{
//		old_Data[2] = data;
//		/* 判断是否更新比较基准，必须在云台与目标装甲中心重叠之后进行，不然数据会出现问题 */
//		if(abs(nuc->CenterX.f - nuc->offset_X) < MIN_CENTER_ERR_TO_SHOOT)
//		{
//			if((abs(data - old_Data[1] )< 0.3f) && (abs(old_Data[1] - old_Data[0]) < 0.3f))
//				compare_base = data;
//		}
//		count = 0;
//	}
//	data = data - compare_base;
//	last_centerX = nuc->CenterX.f;
//	return data;
//}
