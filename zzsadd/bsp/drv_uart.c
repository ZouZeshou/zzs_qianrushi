#include "drv_uart.h"
#include "drv_dbus.h"
#include "drv_nuc_interface.h"
uint8_t Usart1buff[30]={0};
uint8_t Usart2buff[30]={0};
uint8_t Usart3buff[30]={0};
uint8_t Uart4buff[30]={0};
uint8_t Usart6buff[30]={0};
uint8_t Uart7buff[30]={0};
uint8_t Uart8buff[30]={0};
uint8_t dbus_decode_flag = 0;
uint8_t vision_decode_flag = 0;
uint8_t debug_decode_flag = 0;
/**
 * @brief Enable USART
 * @param None
 * @return None
 * @attention None
 */
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t buff_size)
{
	HAL_UART_Receive_IT(huart,buffer_addr,buff_size);
	__HAL_UART_ENABLE_IT(huart,UART_IT_ERR);	
}
/**
* @brief usart send char
* @param argument: Not used
* @retval None
*/
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char)
{
		while((huart->Instance->SR&0X40)==0); 
		huart->Instance->DR = u8_char;
}
/**
 * @brief Enable the Usart DMA
 * @param None
 * @return None
 * @attention  None
 */
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size)
{
	 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);								/*!<使能串口的中断为空闲中断    */
	 HAL_UART_Receive_DMA(huart,buffer_addr,data_size);								/*!<DMA Receive data            */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART1_IDLE_IRQ(void)
{	
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);	//清除标志位和SR，DR寄存器
			get_dbus_data();
		  HAL_UART_DMAStop(&huart1);
			HAL_UART_Receive_DMA(&huart1,Usart1buff,18);//函数中包括重新配置DMA
	}
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART3_IDLE_IRQ(void)
{
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);	//清除标志位和SR，DR寄存器
			HAL_UART_DMAStop(&huart3);
			HAL_UART_Receive_DMA(&huart3,Usart3buff,23);//函数中包括重新配置DMA
	}
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART6_IDLE_IRQ(void)
{
	
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);	//清除标志位和SR，DR寄存器
			HAL_UART_DMAStop(&huart6);
			HAL_UART_Receive_DMA(&huart6,Usart6buff,23);//函数中包括重新配置DMA
	}
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART7_IDLE_IRQ(void)
{
	if(__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart7);	//清除标志位和SR，DR寄存器
			vision_decode_handle(Uart7buff);
			HAL_UART_DMAStop(&huart7);
			HAL_UART_Receive_DMA(&huart7,Uart7buff,23);//函数中包括重新配置DMA
	}
  
}
/**
 * @brief Error Callback function（串口中断回调函数）
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //清除错误标志位，清空SR、DR寄存器
	}
}
/**
 * @brief Redirect function for printf（对printf函数的重定义）
 * @param None
 * @return None
 * @attention  The printf function could not be used without this function
 */
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0); 
	USART2->DR = (uint8_t) ch;      
	return ch;
}

/**
 * @brief rx callbackfunction  （串口接收中断回调函数）
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		Dealdata(Usart2buff[0]);
		__HAL_UART_CLEAR_PEFLAG(&huart2);//清除中断标志位
		HAL_UART_Receive_IT(&huart2,Usart2buff,1);//使能串口2
	}
	else
	{
//		printf("uart7 work\r\n");
//		vision_decode_flag = 1;
//		__HAL_UART_CLEAR_PEFLAG(&huart7);//清除中断标志位
//		HAL_UART_Receive_IT(&huart7,Usart2buff,23);//使能串口2
	}
}
/**
 * @brief deal uart IT
 * @param None
 * @return None
 * @attention None
 */
void deal_uart_IT(void)
{
	if(dbus_decode_flag == 1)
	{
		dbus_decode_flag = 0;
	}
	if(vision_decode_flag == 1)
	{
		vision_decode_flag = 0;
	}
	if(debug_decode_flag == 1)
	{
		debug_decode_flag = 0;
	}
}
