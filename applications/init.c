
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
#include "spi.h"
#include "nrf.h"
#include "iic_hml.h"
#include "mpu9250.h"
#include "ms5611_spi.h"
#include "beep.h"
_SYS_INIT sys_init;
u8 mcuID[3];
u8 ble_imu_force;
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
}

u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	SysTick_Configuration(); 	//滴答时钟
	cpuidGetId();
	#if !USE_VER_3
	I2c_Soft_Init();					//初始化模拟I2C
	#endif
	PWM_Out_Init(400);				//初始化电调输出功能	
	PWM_AUX_Out_Init(50);	
	Delay_ms(100);						//延时
	LED_Init();								//LED功能初始
	Delay_ms(2000);						//启动延时
	#if USE_VER_3
	Beep_Init(0,84-1);//2k=1M/500
	#endif
	Delay_ms(1000);
	SPI2_Init();
	#if USE_VER_3
	Mpu9250_Init();
	ms5611DetectSpi();
	#else
	#if EN_ATT_CAL_FC
	MPU6050_Init(20);
	#if !USE_ZIN_BMP
	HMC5883L_SetUp();
	#else
	IIC_Init();
	#endif
	Delay_ms(100);						//延时
	MS5611_Init_FC();
	#endif
	#endif
	
	
	Cycle_Time_Init();
  Usart1_Init(115200L);			//蓝牙
	if(mcuID[0]==0x2B&&mcuID[1]==0x17&&mcuID[2]==0x31)
	Usart1_Init(38400L);			//3DR
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
  Usart4_Init(256000L);     //接收机  SD卡
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if !USE_ZIN_BMP	
	#if USE_PXY										
	Usart3_Init(115200L);  
	#else
	#if USE_ANO_FLOW
	Usart3_Init(500000L); 
	#else
	Usart3_Init(230400L);     // 未使用 或者 超声波
	#endif
	#endif
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
  #endif
	#endif
	#if USE_MINI_FC_FLOW_BOARD
	Uart5_Init(100000);	
  #else

	#if !SONAR_USE_FC1
  Uart5_Init(115200L);      // 图像Odroid
	#else
	Uart5_Init(9600);      // 超声波
	#endif

	#endif
	Delay_ms(100);
	#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);    
	#endif
	#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     	
	#endif
	#if USE_ZIN_BMP
	#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);  	
	#endif
	#endif
	#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     
	#endif	
	#if SONAR_USE_FC
	#if !USE_ZIN_BMP	
	Ultrasonic_Init();
	#endif
	#endif
	#if !FLASH_USE_STM32	
	W25QXX_Init();		
	if(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到flash
	Delay_ms(100);
	#endif
	READ_PARM();//读取参数
	Para_Init();//参数初始
	#if USE_MINI_FC_FLOW_BOARD||USE_VER_3		
	Nrf24l01_Init(MODEL_RX2,40);
	Nrf24l01_Check();
	#endif
	//TIM3_Int_Init(1000-1,8400-1);
	//-------------系统默认参数
	mode_oldx.en_eso_h_in=1;
	mode_oldx.imu_use_mid_down=1;
	mode_oldx.flow_f_use_ukfm=2;
	mode_oldx.baro_f_use_ukfm=0;				
	mode_oldx.yaw_use_eso=0;
	
	
// Need init for First use mpu6050_fc ak8975_fc	
//  LENGTH_OF_DRONE=330;//飞行器轴距
//  SONAR_HEIGHT=0.054+0.015;//超声波安装高度
//	imu_board.k_flow_sel=1;//光流增益 飞行0.89  志陈 0.5
//	imu_board.flow_module_offset_x=-0.05;//光流安装位置
//	imu_board.flow_module_offset_y=0;//光流安装位置
//	imu_board.flow_set_yaw=0;//光流安装位置
	fan.max[0]=486;//680;
	fan.min[0]=486;//;
	fan.off[0]=3280;
	fan.max[1]=486;//;
	fan.min[1]=486;//;
	fan.off[1]=3180;
	fan.max[2]=486;//;
	fan.min[2]=486;//;
	fan.off[2]=3200;
	fan.max[3]=486;//;
	fan.min[3]=486;//;
	fan.off[3]=3333;
	fan.flag[0]=1;
	fan.flag[1]=1;
	fan.flag[2]=1;
	fan.flag[3]=1;
	fan.per_degree=1;
	#if USE_VER_3
	Delay_ms(1000);
	if(module.acc_imu==2&&module.gyro_imu==2&&module.hml_imu==2&&1)
		Play_Music_Direct(MEMS_RIGHT_BEEP);
	else
		Play_Music_Direct(MEMS_ERROR_BEEP);
	#endif
 	return (1);
}
