#include "drv_adc.h"
#include "adc.h"
extern ADC_HandleTypeDef hadc1;
uint16_t adc1_rx_buffer[2];
float oled_button_voltage = 0.0f;
/**
 * @brief adc init function 
 * @param None
 * @return None
 * @attention None
 */
void adc_init(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_rx_buffer,2);
}
/**
 * @brief adc callback function
 * @param None
 * @return None
 * @attention adc1中断由Tim2周期更新触发
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		static float real_voltage = 0.0f;
		static float test_voltage = 0.0f;
		static float sum1 = 0.0f;
		static float sum2 = 0.0f;
		static uint16_t count1 = 0;

		sum1 += adc1_rx_buffer[0];
		sum2 += adc1_rx_buffer[1];//芯片内基准电压1.2V
		if(count1++ > 100)
		{
			real_voltage = sum1/sum2*1.2f;
			oled_button_voltage = real_voltage;
			count1 = 0;
			sum1 = 0.0f;
			sum2 = 0.0f;
		}
	}
}
