#include "drv_timer.h"
#include "global.h"
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;
/**
 * @brief Enable timer6（使能定时器6）
 * @param None
 * @return None
 * @attention None
 */
void TIM_Enable (void)
{
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim2);// 触发 adc1采集
	HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief  turn on the buzzer 
 * @param None
 * @return None
 * @attention for debug
 */
void Buzzer_on(int16_t arr,int16_t ccr)
{
	BUZZER_ARR = arr;
	BUZZER_CCR = ccr;
}
/**
 * @brief  turn off the buzzer 
 * @param None
 * @return None
 */
void Buzzer_off(void)
{
	BUZZER_CCR =0;
}
/**
 * @brief  open magazine 
 * @param None
 * @return None
 */
void open_bullet_magazine(void)
{
  #if    ROBOT_ID == 1
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1650);
  #elif  ROBOT_ID == 2
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1650);
  #elif  ROBOT_ID == 3
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1650);
  #endif
}
/**
 * @brief  close magazine 
 * @param None
 * @return None
 */
void close_bullet_magazine(void)
{
	#if    ROBOT_ID == 1
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1100);
  #elif  ROBOT_ID == 2
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1100);
  #elif  ROBOT_ID == 3
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1100);
  #endif
}
/**
 * @brief initialise the data pwm 
 * @param None
 * @return None
 * @attention None
 */
void pwm_init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // imu temp
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // buzzer
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);//servo
	Buzzer_off();
  close_bullet_magazine();
}
