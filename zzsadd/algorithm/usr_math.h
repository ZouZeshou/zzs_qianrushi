#ifndef __USR_MATH_H
#define __USR_MATH_H
#include "stm32f4xx_hal.h"
//�жϷ���λ
float sign(float value);
//��������
 float float_deadline(float Value, float minValue, float maxValue);
//int16����
 int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
float float_constrain(float Value, float minValue, float maxValue);
//�޷�����
 int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
 int int_constrain(int Value, int minValue, int maxValue);
//ѭ���޷�����
 float loop_float_constrain(float Input, float minValue, float maxValue);
//�Ƕ� ���޷� 180 ~ -180
float theta_format(float Ang);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)


#endif
