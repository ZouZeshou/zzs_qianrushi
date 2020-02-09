#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "stm32f4xx.h"
#include "stdint.h"
#include "main.h"
void adc_init(void);
extern uint16_t adc1_rx_buffer[2];
extern float oled_button_voltage;
#endif
