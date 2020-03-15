#ifndef DRV_UART__H
#define DRV_UART__H
#include "stm32f4xx.h"
#include "stdio.h"
#include "usart.h"
#include "STMGood.h"

void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t buff_size);
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char);
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size);
void DBUS_IDLE_IRQ(void);
void VISION_IDLE_IRQ(void);
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern uint8_t dbus_buffer[30];
extern uint8_t debug_buffer[30];
extern uint8_t vision_buffer[30];
extern uint8_t dbus_decode_flag ;
extern uint8_t vision_decode_flag ;
extern uint8_t debug_decode_flag ;
extern int fputc(int ch, FILE *f);
void deal_uart_IT(void);
#endif
