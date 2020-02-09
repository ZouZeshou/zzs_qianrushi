#ifndef __DRV_BUZZER_H
#define __DRV_BUZZER_H
#include "tim.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

bool BUSSER_ONCE(void);
void Di(void);
void Di_Di(void);
void Di_Di_Di(void);
void Di_Di_Di_Di(void);
void DoNot_Di(void);
void Biu_biubiu(void);
void play_music(float arr);
#endif
