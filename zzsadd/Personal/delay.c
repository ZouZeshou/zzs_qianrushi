#include "delay.h"
/*Include the header file which are the system product automatically*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "main.h"
#include "can.h"
/*Include the header file which are frome C lib*/
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
static uint32_t fac_us=0;

void xdelay_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=90000;
		while(a--);
	}
}

void xdelay_us(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}

void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}

void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	taskENTER_CRITICAL();
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ   				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	}
	taskEXIT_CRITICAL();
}
