#include "drv_uart.h"
#include "drv_dbus.h"
#include "drv_nuc_interface.h"
#include "global.h"
uint8_t dbus_buffer[30]={0};
uint8_t debug_buffer[30]={0};
uint8_t vision_buffer[30]={0};
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
	 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);								/*!<ʹ�ܴ��ڵ��ж�Ϊ�����ж�    */
	 HAL_UART_Receive_DMA(huart,buffer_addr,data_size);								/*!<DMA Receive data            */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
}
/**
 * @brief Interrupt function for dbus
 * @param None
 * @return None
 * @attention None
 */
void DBUS_IDLE_IRQ(void)
{	
	if(__HAL_UART_GET_FLAG(&DBUS_HANDLE,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HANDLE);	//�����־λ��SR��DR�Ĵ���
			get_dbus_data();
		  HAL_UART_DMAStop(&DBUS_HANDLE);
			HAL_UART_Receive_DMA(&DBUS_HANDLE,dbus_buffer,DBUS_LENGTH);//�����а�����������DMA
	}
}
/**
 * @brief Interrupt function for vision
 * @param None
 * @return None
 * @attention None
 */
void VISION_IDLE_IRQ(void)
{
	if(__HAL_UART_GET_FLAG(&VISION_HANDLE,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&VISION_HANDLE);	//�����־λ��SR��DR�Ĵ���
			vision_decode_handle(vision_buffer);
			HAL_UART_DMAStop(&VISION_HANDLE);
			HAL_UART_Receive_DMA(&VISION_HANDLE,vision_buffer,VISION_LENGTH);//�����а�����������DMA
	}
  
}
/**
 * @brief Error Callback function�������жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //��������־λ�����SR��DR�Ĵ���
	}
}
/**
 * @brief Redirect function for printf����printf�������ض��壩
 * @param None
 * @return None
 * @attention  The printf function could not be used without this function
 */
int fputc(int ch, FILE *f)
{ 	
	while((DEBUG_URAT->SR&0X40)==0); 
	DEBUG_URAT->DR = (uint8_t) ch;      
	return ch;
}

/**
 * @brief rx callbackfunction  �����ڽ����жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == DEBUG_URAT)
	{
		Dealdata(debug_buffer[0]);
		__HAL_UART_CLEAR_PEFLAG(&DEBUG_HANDLE);//����жϱ�־λ
		HAL_UART_Receive_IT(&DEBUG_HANDLE,debug_buffer,DEBUG_LENGTH);//ʹ�ܴ���2
	}
	else
	{

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
