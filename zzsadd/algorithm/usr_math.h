#ifndef __USR_MATH_H
#define __USR_MATH_H
#include "stm32f4xx_hal.h"
//判断符号位
float sign(float value);
//浮点死区
 float float_deadline(float Value, float minValue, float maxValue);
//int16死区
 int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
float float_constrain(float Value, float minValue, float maxValue);
//限幅函数
 int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
 int int_constrain(int Value, int minValue, int maxValue);
//循环限幅函数
 float loop_float_constrain(float Input, float minValue, float maxValue);
//角度 °限幅 180 ~ -180
float theta_format(float Ang);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)


#endif
