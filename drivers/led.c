
#include "led.h"
#include "include.h"
#include "mymath.h"

void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
#if USE_MINI_BOARD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif
	LEDRGB();
	Delay_ms(1000);
	LEDRGB();
}

void LEDRGB(void)
{static u8 flag;
#if USE_MINI_BOARD
if(!flag){flag=1;
GPIO_ResetBits(GPIOC,GPIO_Pin_1);}
else{flag=0;
GPIO_SetBits(GPIOC,GPIO_Pin_1);}
#else
if(!flag){flag=1;
GPIO_ResetBits(GPIOD,GPIO_Pin_12);}
else{flag=0;
GPIO_SetBits(GPIOD,GPIO_Pin_12);}
#endif
}


