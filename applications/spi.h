#ifndef __SPI_H
#define __SPI_H
#include "include.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

				    
// SPI�����ٶ����� 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7
						  	    													  
void SPI2_Init(void);			 //��ʼ��SPI1��
u8 Spi_RW(u8 TxData);//SPI1���߶�дһ���ֽ�

#define MPU9250 0
#define NRF2401 1
#define MS5611  2
void SPI_CS(u8 sel,u8 set);

#define SPI_CE_H()   GPIO_SetBits(GPIOC, GPIO_Pin_2) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOC, GPIO_Pin_2)

#define SPI_CSN_H()  SPI_CS(NRF2401,1)//GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_L()  SPI_CS(NRF2401,0)//GPIO_ResetBits(GPIOB, GPIO_Pin_12) 


#endif

