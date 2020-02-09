#include "drv_buzzer.h"
bool BUSSER_ONCE(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 20)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 20 && count < 34 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 34 && count < 42 )
	{
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
		return 1;
	}
	else
		return 0;
} 	

void Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 62 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);
	else if(count > 62)
	count = 0;

}

void Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 68)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	else if(count > 68)
	count = 0;
}

void Di_Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 12)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 12 && count < 14 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 14 && count < 74 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	else if(count > 74)
	count = 0;
}

void Di_Di_Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 12)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 12 && count < 14 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 14 && count < 18 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 18 && count < 20 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 20 && count < 60)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	else if(count > 60)
	count = 0;
}

void DoNot_Di(void)
{
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
}

void Biu_biubiu(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 3 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 3 && count < 5 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 5 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 10 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);
	if(count > 10 && count < 13 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 13 && count < 24 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);		
	else if(count > 24)
	count = 0;
}
/***copy from qinsuo********************/
#define MUSIC 2
#if MUSIC == 0
//  ?? :
int tune[]={1276,1517,1276,956,1136,956,1276,
1276,1912,1704,1517,1704,1912,1704,
1276,1517,1276,956,1012,1136,956,1276,
1276,1704,1517,1433,2024,1912,
1136,956,956,1012,1136,1012,956,
1136,1012,956,1136,1136,1276,1517,1912,1706,
1276,1517,956,1012,1136,956,1276,
1276,1704,1517,1433,2024,1912};
int duration[]={1568,659,784,4184,1760,2092,3136,
1568,523,587,1318,587,523,3511,
1568,659,784,3138,988,1760,2092,3136,
1568,587,659,2094,494,3661,
1760,2092,4184,1976,880,988,4184,
880,988,1046,880,880,784,659,523,3522,
1568,1318,3138,988,1760,2092,3136,
1568,587,659,2094,494,3138};	
#elif  MUSIC == 1
int tune[]={1517,1517,1517,1912,1517,0,1276,0,2551,
0,1912,0,2551,0,3030,0,2273,
2024,2024,2273,2551,1517,1276,1136,0,1433,1276,
0,1517,0,1912,1704,2024,0,1912,0,2551,0,
0,3030,0,2273,2024,0,2024,2273,0,2551,1517,
1276,1136,1433,1276,0,1517,1704,1912,2024,
0,1276,1433,1433,1517,1517,2551,2273,1912,
2273,1912,1704,0,1276,1433,1433,1517,0,1517,956,
956,956,0,0,1276,1433,1433,1517,1517,
0,2551,0,2273,1912,2273,1912,0,1704,0,1517,0,
1704,0,1912,0,0,0,1276,
0,1433,1433,1517,0,1517,0,2551,2273,1912,0,2273,1912,1704
,0,1276,1433,1433,1517,0,1517,0,1912,0,1912,1912,
0,0,1276,1433,1433,1517,0,1517,2551,2273,
0,1912,0,2273,1912,1704,0,0,1517,0,0,1517,0,0,1704,0,
1912,0,0,0,1912,1912,0,1912,0,
1912,1704,0,1517,0,1912,0,2273,2551,0,1912,1912,
0,1912,0,1912,1704,1517,0,0,
1912,1912,0,1912,1912,1704,0,1517,1912,0,2273,2551,
0,1517,1517,0,1517,0,1912,1517,0,1276,0,	
0,2551,0,1912,0,2551,0,3030,
0,2273,2024,2024,2273,2551,1517,1433,0,1136,
0,2273,1276,0,1517,0,1912,1704,2024,0,1912,0,
0,2273,1276,0,1517,0,1912,1704,2024,2273,
2551,1517,1276,1136,0,1433,1276,0,1517,0,1912,1704,
2024,0,1517,1912,0,2551,2551,2273,1443,
0,1433,2273,0,2024,1136,1136,1136,1276,0,1433,
1704,1912,2273,2551,0,0,1517,1912,0,2551,
0,2551,2273,1433,0,1433,2273,0,2024,1433,
0,1433,1433,1517,1704,0,1912,3030,0,3030,3817
};
int duration[] = {659,659,659,262,330,600,784,600,392,
600,262,600,196,600,330,600,440,
494,247,440,196,659,392,440,600,349,392,
600,330,600,262,294,494,600,523,600,196,
600,600,165,440,247,600,247,440,600,196,659,
784,880,698,784,600,659,294,262,988,
600,1568,349,349,659,659,196,440,523,600,
220,262,587,600,392,349,698,659,600,659,1046,
1046,600,600,392,349,349,659,330,
600,196,600,264,523,220,262,600,294,600,659,600,
587,600,523,600,600,600,784,
600,349,349,330,600,330,600,196,220,523,600,220,262,294,
600,392,349,349,659,600,330,600,262,600,523,523,	
600,600,392,349,349,330,600,659,196,220,
600,262,600,220,262,294,600,600,330,600,600,330,600,600,294,600,
523,600,600,600,262,262,600,262,
262,294,600,330,600,262,600,220,584,600,262,523,
600,262,600,262,294,330,600,600,
262,262,600,523,262,294,600,330,262,600,440,392,
600,330,330,600,330,600,262,330,600,392,600,
600,392,600,523,600,392,600,330,
600,440,494,494,440,392,330,349,600,440,
600,220,784,600,330,600,262,587,494,600,262,600,
600,392,600,660,440,494,600,494,440,
392,659,392,440,600,349,392,600,330,600,262,294,
494,600,330,262,600,196,196,220,698,
600,349,440,600,247,880,880,440,784,600,1396,
587,523,440,392,600,600,330,262,600,392,
600,196,220,349,600,349,880,600,247,698,
600,349,698,330,294,600,262,330,600,165,262,600
};
#elif  MUSIC == 2
int tune[]={1517,1517,1517,1912,1517,0,1276,0,2551};
int duration[] = {659,659,659,262,330,600,784,600,392};
#else
int tune[] = {1136,1136,956,851,715,715,851,956,956,851,956,
1136,1136,956,851,715,715,851,956,956,851,956,
956,956,956,1136,956,851,851,956,
1136,1275,1136,956,1136,1275,1432,1432,1275,1432,
1136,1275,1432,1136,1275,1136,956,851,715,956,
1275,1136,956,1275,1136,1432,1703,1912,
1703,1432,1275,1136,1432,1275,1432,1703,1912};
int duration[] = {2*528,2*264,2*313,2*352,2*419,2*419,2*352,2*627,2*313,2*352,2*1255,
2*528,2*264,2*313,2*352,2*419,2*419,2*352,2*627,2*313,2*352,2*1255,
2*627,2*627,2*627,2*264,2*313,2*705,2*705,2*1255,
2*528,2*235,2*264,2*627,2*264,2*235,2*418,2*209,2*235,4*418,
2*264,2*235,2*209,2*264,2*705,2*264,2*627,2*352,2*419,2*1255,
2*470,2*264,2*313,2*235,2*264,2*209,2*176,2*627,
2*352,2*418,2*705,2*264,2*209,2*325,2*209,2*176,1.5*1255};
#endif
uint32_t TIM12_ARR = 1200;
int Length = sizeof(tune)/sizeof(tune[0]);
void play_music(float arr)
{
	static uint8_t void__flag;
  for(int i = 0; i < Length; ++i)
	{
		if(void__flag)
		{
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			void__flag = 0;
		}
		if(tune[i] == 0)
		{
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
			void__flag = 1;
		}
		else
		{
			TIM12_ARR = tune[i];
			__HAL_TIM_SET_AUTORELOAD(&htim12,TIM12_ARR);
		}
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,TIM12_ARR/2);		
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);	  	
		HAL_Delay(arr*duration[i]*0.35f);
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);				
	}
}