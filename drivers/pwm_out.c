

#include "pwm_out.h"
#include "include.h"
#include "mymath.h"
//21分频到 84000000/21 = 4M   0.25us

#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0
u8 motor_cal=0;//电调校准标志位
void 	MOTOR_SET(void)
{
#if USE_MINI_BOARD
Delay_ms(1000);
 	TIM3->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//5	
 	TIM3->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//6	
  TIM3->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM3->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
	TIM4->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM4->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
Delay_ms(2000);
	TIM3->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
 	TIM3->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
  TIM3->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM3->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	TIM4->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM4->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
Delay_ms(100);	
#else	
Delay_ms(4000);
 	TIM1->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//5	
 	TIM1->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//6	
  TIM1->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM1->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
Delay_ms(4000);
	TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
 	TIM1->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
  TIM1->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM1->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
#endif
	
}
u8 PWM_Out_Init(uint16_t hz)//400hz
{
	#if USE_MINI_BOARD
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOB, ENABLE);

		//////////////////////////////////TIM3///////////////////////////////
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); 
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

		 /* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel3 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel4 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_CtrlPWMOutputs(TIM3, ENABLE);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
		TIM_Cmd(TIM3, ENABLE);	

		//////////////////////////////////////TIM4///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);

		TIM_CtrlPWMOutputs(TIM4, ENABLE);
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);	

		if(motor_cal)//《---------------在此处加断点 校准电调
		MOTOR_SET();
		TIM3->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
		TIM3->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
		TIM3->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM3->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
		TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
		TIM4->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM4->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	

		if( hz_set > 84000000 )
		{
		return 0;
		}
		else
		{
		return 1;
		}
	
	
	#else
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);
		
		hz_set = LIMIT (hz_set,1,84000000);
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOB, ENABLE);

	//////////////////////////////////TIM1///////////////////////////////
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOE, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
		
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

		 /* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel3 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel4 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_ARRPreloadConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);	
			
		//////////////////////////////////////TIM4///////////////////////////////////////////

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		 /* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_CtrlPWMOutputs(TIM4, ENABLE);
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);	

	if(motor_cal)//《---------------在此处加断点
		MOTOR_SET();
		TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
		TIM1->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
		TIM1->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM1->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
		TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	

		if( hz_set > 84000000 )
		{
			return 0;
		}
		else
		{
			return 1;
		}
	#endif
	
}

u8 PWM_AUX_Out_Init(uint16_t hz)//50Hz
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	
		//////////////////////////////////////TIM8///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOC, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM8, &TIM_OCInitStructure);


		TIM_CtrlPWMOutputs(TIM8, ENABLE);
		TIM_ARRPreloadConfig(TIM8, ENABLE);
		TIM_Cmd(TIM8, ENABLE);	

    SetPwm_AUX(0,0);
		
	
}

AUX_S aux;
void SetPwm_AUX(float pit,float rol)
{ static u8 init;
	u8 i;
	if(!init)
	{
	init=1;
	aux.init[0]=1550;
	aux.init[1]=1625;
	aux.min[0]=500;
	aux.min[1]=500;
  aux.max[0]=2500;
	aux.max[1]=2500;
	aux.flag[0]=1;
  aux.flag[1]=1;		
	aux.pwm_per_dig=5;
	}	
	aux.pwm_tem[0]=aux.init[0]+aux.pwm_per_dig*pit*aux.flag[0];
	aux.pwm_tem[1]=aux.init[1]+aux.pwm_per_dig*rol*aux.flag[1];
	for(i=0;i<2;i++)
	{
			aux.pwm_tem[i] = LIMIT(aux.pwm_tem[i],aux.min[i],aux.max[i]);
	}
	
	TIM8->CCR1 = (aux.pwm_tem[0] ) ;				//1	
	TIM8->CCR2 = (aux.pwm_tem[1] ) ;				//2
}	

u16 BEEP_RATE;
u8 PMW_T=1;
#if USE_MINI_BOARD
u8 CH_out_Mapping[8] = {0,1,2,3,4,5,6,7};
#else
u8 CH_out_Mapping[MAXMOTORS] = {0,1,2,3};
#endif
void CH_out_Mapping_Fun(u16 *out,u16 *mapped )
{
	u8 i;
	for( i = 0 ; i < MAXMOTORS ; i++ )
	{
		*( mapped + i ) = *( out + CH_out_Mapping[i] );
	}
}
u8 sel=0;
void SetPwm(int16_t pwm[MAXMOTORS],s16 min,s16 max)
{
	u8 i;
	#if USE_MINI_BOARD
	s16 pwm_tem[8];
  #else
	s16 pwm_tem[MAXMOTORS];
	#endif
	for(i=0;i<MAXMOTORS;i++)
	{
			pwm_tem[i] = pwm[i] ;
			pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
	}
	
#if USE_MINI_BOARD
	TIM3->CCR1 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//1	
	TIM3->CCR2 = PWM_RADIO *( pwm_tem[1] ) + INIT_DUTY;				//2
	TIM3->CCR3 = PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//3	
	TIM3->CCR4 = PWM_RADIO *( pwm_tem[3] ) + INIT_DUTY;				//4
	TIM4->CCR1 = PWM_RADIO *( pwm_tem[4] ) + INIT_DUTY;				//3	
	TIM4->CCR2 = PWM_RADIO *( pwm_tem[5] ) + INIT_DUTY;				//4
	TIM4->CCR3 = PWM_RADIO *( pwm_tem[6] ) + INIT_DUTY;				//3	
	TIM4->CCR4 = PWM_RADIO *( pwm_tem[7] ) + INIT_DUTY;				//4
	#if USE_BEEP
	TIM4->CCR2 = PWM_RADIO *( BEEP_RATE ) + INIT_DUTY;				//4
	#endif
#else
	TIM1->CCR4 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//1	
	TIM1->CCR2 = PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//2
	TIM1->CCR3 = PWM_RADIO *( pwm_tem[1] ) + INIT_DUTY;				//3	
	TIM1->CCR1 = PWM_RADIO *( pwm_tem[3] ) + INIT_DUTY;				//4
	TIM4->CCR1 = PWM_RADIO *( pwm_tem[4] ) + INIT_DUTY;				//3	
	TIM4->CCR2 = PWM_RADIO *( pwm_tem[5] ) + INIT_DUTY;				//4
#endif
}

