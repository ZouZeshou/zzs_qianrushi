#ifndef DEBUG_PRINTF_H
#define DEBUG_PRINTF_H
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "imu_update_task.h"
#include "infantry.h"
#include "stm32f4xx_hal.h"
void printf_debug_data(uint8_t time_gap);
void printf_chassis(void);
void printf_gimbal(void);
void printf_shoot(void);
void print_debus(void);
void show_fps_data(void);
void printf_judge(void);
void print_vision(void);
#endif
