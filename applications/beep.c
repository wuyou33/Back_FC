#include   "beep.h"
#include   "time.h"


u8 start_music[]={
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 17,21,
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 17,21, 21,21,
};

u8 mems_right_music[]={
 05,34, 06,34, 07,34, 
};

u8 mems_error_music[]={
 07,37, 00,32, 07,36, 00,32, 
};

u8 start_music_pix[]={
 04,32, 24,22, 05,32, 
 04,32, 24,22, 05,32,
 04,32, 24,22, 05,32,
};
u8 test_tone=24;
u16 Beat_delay[7]={0,62,94,125,125,187,250};
void Play_Music(u8 *music, u16 st,u16 ed)
{
	u16 i;
  for(i=st;i<ed-st;i++)
   {
		 u8 level=music[i*2]/10;
		 u8 tone=music[i*2]%10;
		 Tone(level,tone);
		 u8 loop=music[i*2+1]%10;
		 u8 beat=music[i*2+1]/10;
     Delay_ms(Beat_delay[beat]*loop);	 
	 }	
   Tone(0,0);		 
}

u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,float dt)
{
	u16 i;
  for(i=st;i<ed-st;i++)
   {
		 u8 level=music[i*2]/10;
		 u8 tone=music[i*2]%10;
		 Tone(level,tone);
		 u8 loop=music[i*2+1]%10;
		 u8 beat=music[i*2+1]/10;
     Delay_ms(Beat_delay[beat]*loop);	 
	 }	
   Tone(0,0);		 
}

void Play_Music_Direct(u8 sel)
{
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music(mems_right_music,0,sizeof(mems_right_music)/2);	
	  break;
		case START_BEEP:
		Play_Music(start_music,0,sizeof(start_music)/2);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music(mems_error_music,0,sizeof(mems_error_music)/2);		
		break;
	}
}

void Beep_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); //GPIOF9����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM14
  Tone(0,0);
		
		
	Play_Music_Direct(START_BEEP);
//  u8 i,j;
//	for(i=0;i<3;i++)
//	 for(j=1;j<8;j++)
//	 {Tone(i,j);Delay_ms(111);}
//	 Tone(0,0);
}  

u16 tone_table[3][8]={
     {0,261,293,329,349,391,440,493},
     {0,523,587,659,698,783,880,987},
     {0,1046,1174,1318,1396,1567,1760,1975}};
void Tone(u8 level, u8 tone)
{ u32 psc=84-1;
  u32 arr=1000000/tone_table[level][tone]-1;
  if(tone==0)
		arr=1000000/1-1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM14
//	TIM8->PSC=psc;
//	TIM8->ARR=arr;
//	TIM8->CCR4=arr/2;

}
