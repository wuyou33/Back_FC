
#include "include.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"
#include "ctrl.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "bmp.h"

u8 mcuID[3];
u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 	//滴答时钟
	
	I2c_Soft_Init();					//初始化模拟I2C
	Delay_ms(100);						//延时
	PWM_Out_Init(400);				//初始化电调输出功能	
	#if EN_ATT_CAL_FC
	MPU6050_Init(20);   			//加速度计、陀螺仪初始化，配置20hz低通
	#endif
	LED_Init();								//LED功能初始化
	
	Para_Init();							//参数初始化
	
	Delay_ms(100);						//延时
	
	#if EN_ATT_CAL_FC
	HMC5883L_SetUp();
	Delay_ms(100);						//延时
	MS5611_Init_FC();
	#endif
	
	Cycle_Time_Init();
  Usart1_Init(115200L);			//蓝牙
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
  Usart4_Init(576000L);     //接收机  SD卡
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if USE_PXY										
	Usart3_Init(115200L);  
	#else
	Usart3_Init(230400L);     // 未使用
	#endif
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
  #endif
  Uart5_Init(115200L);      // 图像Odroid
	Delay_ms(100);
	#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);    
	#endif
	#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     	
	#endif
	#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);  	
	#endif
	#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     
	#endif	
	#if SONAR_USE_FC
	Ultrasonic_Init();
	#endif
	#if !FLASH_USE_STM32	
	W25QXX_Init();		
	while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到flash
	Delay_ms(100);
	#endif
	READ_PARM();//读取参数
 	return (1);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
