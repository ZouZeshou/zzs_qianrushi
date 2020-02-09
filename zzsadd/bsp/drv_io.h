#ifndef __DRV_IO_H
#define __DRV_IO_H
#include "tim.h"

#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
#define IST_ENABLE_Pin GPIO_PIN_2
#define IST_ENABLE_GPIO_Port GPIOE
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define VCC_OUT_Pin GPIO_PIN_2
#define VCC_OUT_GPIO_Port GPIOH
#define VCC_OUTH3_Pin GPIO_PIN_3
#define VCC_OUTH3_GPIO_Port GPIOH
#define VCC_OUTH4_Pin GPIO_PIN_4
#define VCC_OUTH4_GPIO_Port GPIOH
#define LED_H_Pin GPIO_PIN_8
#define LED_H_GPIO_Port GPIOG
#define VCC_OUTH5_Pin GPIO_PIN_5
#define VCC_OUTH5_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOG
#define LED_F_Pin GPIO_PIN_6
#define LED_F_GPIO_Port GPIOG
#define LED_E_Pin GPIO_PIN_5
#define LED_E_GPIO_Port GPIOG
#define LED_D_Pin GPIO_PIN_4
#define LED_D_GPIO_Port GPIOG
#define LED_C_Pin GPIO_PIN_3
#define LED_C_GPIO_Port GPIOG
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOG
#define LED_A_Pin GPIO_PIN_1
#define LED_A_GPIO_Port GPIOG
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
void turn_on_laser(void);
void turn_off_laser(void);
void io_init(void);


#endif
