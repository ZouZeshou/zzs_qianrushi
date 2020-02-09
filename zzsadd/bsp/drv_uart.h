#ifndef DRV_UART__H
#define DRV_UART__H
#include "stm32f4xx.h"
#include "stdio.h"
#include "usart.h"
#include "STMGood.h"
void USART1_DMA_Enable(void);
void USART6_DMA_Enable(void);
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t buff_size);
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char);
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size);
void USART1_IDLE_IRQ(void);
void USART6_IDLE_IRQ(void);
void USART3_IDLE_IRQ(void);
void USART7_IDLE_IRQ(void);
void USART8_IDLE_IRQ(void);
extern uint8_t Usart1buff[30];
extern uint8_t Usart2buff[30];
extern uint8_t Usart3buff[30];
extern uint8_t Uart4buff[30];
extern uint8_t Usart6buff[30];
extern uint8_t Uart7buff[30];
extern uint8_t Uart8buff[30];
extern uint8_t dbus_decode_flag ;
extern uint8_t vision_decode_flag ;
extern uint8_t debug_decode_flag ;
extern int fputc(int ch, FILE *f);
void deal_uart_IT(void);
#endif
