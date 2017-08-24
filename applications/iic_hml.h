#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f4xx.h"
#include "include.h"

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 
   
//#if USE_MINI_FC_FLOW_BOARD
#define PIN_SCL_F 5
#define PIN_SDA_F 4
//#else
#define PIN_SCL 4
#define PIN_SDA 5

#define PIN_SCL_T 11
#define PIN_SDA_T 10
//#endif


//IO��������	 
#define IIC_SCL_F    PCout(PIN_SCL_F) //SCL
#define IIC_SDA_F    PCout(PIN_SDA_F) //SDA	 
#define READ_SDA_F   PCin(PIN_SDA_F)  //����SDA 

//IO��������	 
#define IIC_SCL_T    PBout(PIN_SCL_T) //SCL
#define IIC_SDA_T    PBout(PIN_SDA_T) //SDA	 
#define READ_SDA_T   PBin(PIN_SDA_T)  //����SDA 
//IO��������	 
#define IIC_SCL    PCout(PIN_SCL) //SCL
#define IIC_SDA    PCout(PIN_SDA) //SDA	 
#define READ_SDA   PCin(PIN_SDA)  //����SDA 


#define SDA_IN()  {GPIOC->MODER&=~(3<<(PIN_SDA*2));GPIOC->MODER|=0<<PIN_SDA*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOC->MODER&=~(3<<(PIN_SDA*2));GPIOC->MODER|=1<<PIN_SDA*2;} //PB9���ģʽ

#define SDA_IN_F()  {GPIOC->MODER&=~(3<<(PIN_SDA_F*2));GPIOC->MODER|=0<<PIN_SDA_F*2;}	//PB9����ģʽ
#define SDA_OUT_F() {GPIOC->MODER&=~(3<<(PIN_SDA_F*2));GPIOC->MODER|=1<<PIN_SDA_F*2;} //PB9���ģʽ

#define SDA_IN_T()  {GPIOB->MODER&=~(3<<(PIN_SDA_T*2));GPIOB->MODER|=0<<PIN_SDA_T*2;}	//PB9����ģʽ
#define SDA_OUT_T() {GPIOB->MODER&=~(3<<(PIN_SDA_T*2));GPIOB->MODER|=1<<PIN_SDA_T*2;} //PB9���ģʽ


//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);
void read_zin(void);


#define IIC_RCC       RCC_APB2Periph_GPIOB
#define IIC_GPIO      GPIOB
#define SCL_PIN       GPIO_Pin_11
#define SDA_PIN       GPIO_Pin_10

#define SCL_H         GPIO_SetBits(GPIOB , SCL_PIN)  
#define SCL_L         GPIO_ResetBits(GPIOB , SCL_PIN) 

#define SDA_H         GPIO_SetBits(GPIOB , SDA_PIN)   
#define SDA_L          GPIO_ResetBits(GPIOB , SDA_PIN) 

#define SCL_read      GPIO_ReadInputDataBit(GPIOB , SCL_PIN) 
#define SDA_read      GPIO_ReadInputDataBit(GPIOB , SDA_PIN) 
#endif

//------------------End of File----------------------------
