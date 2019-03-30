#include <bsp_tim.h>


void BSP_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//频率为180M/4=45M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);//90M
	
//	//TIM5_OC4 摩擦轮电机
//	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
//	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
//	TIM_TimeBaseInitStructure.TIM_Period					=20000-1;
//	TIM_TimeBaseInitStructure.TIM_Prescaler					=45-1;/*   45M/45=1M(1us)  20000*1us=0.02s*/
//	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	//TIM5_OC2 直流推杆左电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=1000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler					=4500-1;/*   45M/45000=1k*/
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 0;
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM5 , TIM_OCPreload_Disable);
	TIM_Cmd(TIM5,ENABLE);
	
	//TIM4_OC1 直流推杆右电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=1000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler					=4500-1;/*   45M/45000=1k(0.1ms)  1000*0.1ms=100ms	*/
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 0;
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM4 , TIM_OCPreload_Disable);
	TIM_Cmd(TIM4,ENABLE);
	
	//TIM2_OC1 辅助轮左电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=1000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler					=4500-1;/*   45M/4500=10k(0.1ms)  1000*0.1ms=100ms	*/
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 0;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2 , TIM_OCPreload_Enable);
	TIM_Cmd(TIM2,ENABLE);
	
	//TIM8_OC2 辅助轮右电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=1000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler					=9000-1;/*   45M/4500=10k(0.1ms)  1000*0.1ms=100ms	*/
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 0;
	TIM_OC2Init(TIM8,&TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM8 , TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	TIM_Cmd(TIM8,ENABLE);
	
	
	
}
