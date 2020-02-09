#ifndef __DRV_TIMER_H
#define __DRV_TIMER_H
#include "tim.h"
#define BUZZER_CCR             TIM12->CCR1
#define BUZZER_ARR           TIM12->ARR
void TIM_Enable (void);
void open_bullet_magazine(void);
void close_bullet_magazine(void);
void pwm_init(void);
#endif
