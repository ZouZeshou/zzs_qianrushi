#include "drv_io.h"
//LED GREEN PF14 RED PE11

/**
 * @brief turn_on_laser 
 * @param None
 * @return None
 * @attention None
 */
void turn_on_laser(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
}
/**
 * @brief turn_off_laser
 * @param None
 * @return None
 * @attention None
 */
void turn_off_laser(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
}

/**
 * @brief initialise the voltage io
 * @param None
 * @return None
 * @attention None
 */
void io_init(void)
{
	turn_on_laser();
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
}
