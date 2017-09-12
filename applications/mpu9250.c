#include "spi.h"
#include "mpu9250.h"
#include "time.h"
#if USE_VER_3
#define MPU9250_CS PBout(8)			//====MPU9250��Ƭѡλ
/**
  ******************************************************************************
  * File Name          : Mpu9250.c
  * Description        : Initialize and update MPU data
  ******************************************************************************
  *	Author						 : AGKODY
  * Date							 : 20/11/2016
  ******************************************************************************
  */
	
#define DELAY 300		//��ʼ����������ٶ�
#define HOLDON 1		//��ȡʱ���Լӿ��ٶ�

int16andUint8_t rawAccel[3];
int16andUint8_t rawGyro[3];
int16andUint8_t rawMag[3];

int16andUint8_t rawMPU6050Temperature;
int16_t orientationMatrix[9];

uint8_t Mpu9250_Init(void)
{
		MPU9250_Write_Reg(USER_CTRL,0X01); 										//====��λ���мĴ���������ִ����һ��������λ��Դ��MPU�ڲ���I2C����ִ��
		Delay_us(DELAY);																			//====��Щ�ط���Ҫ��ʱ�����ʱ���ɴ��ʼ����һЩ
		MPU9250_Write_Reg(PWR_MGMT_1,0X80);  									//====��Դ����,��λMPU9250
		Delay_us(DELAY);
		MPU9250_Write_Reg(CONFIG,0x04);												//====��ͨ�˲��� 0x06 5hz
/**********************Init SLV0 i2c**********************************/	
		MPU9250_Write_Reg(INT_PIN_CFG,0X30);  								//====INT Pin / Bypass Enable Configuration  
		Delay_us(DELAY);
		MPU9250_Write_Reg(INT_ENABLE,0X01);
		Delay_us(DELAY);
		MPU9250_Write_Reg(SMPLRT_DIV,0x00);										//====������1000/(1+0)=1000HZ
		Delay_us(DELAY);
		MPU9250_Write_Reg(I2C_MST_CTRL,0X5D); 								//====I2C MAster mode and Speed 400 kHz
		Delay_us(DELAY);
		MPU9250_Write_Reg(USER_CTRL,0X30); 										//====ʹ��MPU9250SPI
		Delay_us(DELAY);
/*******************Init GYRO and ACCEL******************************/		
		MPU9250_Write_Reg(SMPLRT_DIV, 0x00);  								//====�����ǲ����ʣ�����ֵ��0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
		Delay_us(DELAY);
		MPU9250_Write_Reg(GYRO_CONFIG,0X18);  								//====�����ǲ�����Χ 0X18 ����2000��
		Delay_us(DELAY);
		MPU9250_Write_Reg(ACCEL_CONFIG,0x10); 								//====���ٶȼƲ�����Χ 0X18 ����8g//(0x00 +-2g;)  ( 0x08 +-4g;)  (0x10 +-8g;)  (0x18 +-16g)
		Delay_us(DELAY);
		MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x04);//0x08);							//====���ټƸ�ͨ�˲�Ƶ�� ����ֵ ��0x08  ��1.13kHz��	
		Delay_ms(1);
/**********************Init MAG *************************************/	
	
		MPU9250_Write_Reg(I2C_MST_CTRL, 0x5D);							
		Delay_us(DELAY);
		MPU9250_Write_Reg(I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
		Delay_us(DELAY);
		MPU9250_Write_Reg(I2C_SLV0_REG, AK8963_ST1);
		Delay_us(DELAY);
		MPU9250_Write_Reg(I2C_SLV0_CTRL,I2C_SLVx_EN | 8);			//====enable IIC	and EXT_SENS_DATA==8 Bytes
		Delay_us(DELAY);
		MPU9250_Write_Reg(I2C_SLV4_CTRL, 0x09);
		Delay_us(DELAY);
		MPU9250_Write_Reg(I2C_MST_DELAY_CTRL, 0x81);					//====�����豸0���ݲɼ���ʱ
		Delay_us(DELAY);

//-----------------��printfΪ����MPU92_Mag_WriteReg()���ϵ�ʱ������������
//		printf("1st 1st 1st \n");					
//		printf("I2C_SLV4_ADDR: %d \n",MPU9250_Read_Reg(I2C_SLV4_ADDR));
//		printf("I2C_SLV4_REG: %d \n",MPU9250_Read_Reg(I2C_SLV4_REG));
//		printf("I2C_SLV4_DO: %d \n",MPU9250_Read_Reg(I2C_SLV4_DO));
//		printf("I2C_SLV4_CTRL: %d \n",MPU9250_Read_Reg(I2C_SLV4_CTRL));
//		printf("I2C_MST_STATUS: %d \n",MPU9250_Read_Reg(I2C_MST_STATUS));
//		printf("I2C_MST_STATUS: %d \n",MPU9250_Read_Reg(I2C_MST_STATUS));
//		printf("AK8963_ASAX: %d \n",MPU92_Mag_ReadReg(AK8963_ASAX));
		
		
		
		
		MPU92_Mag_WriteReg(AK8963_CNTL2,0x01); // Reset AK8963
		Delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x00); //Power-down mode
		Delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x0F); //Fuse ROM access mode  	
		Delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x00); //Power-down mode
		Delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x06);//0x06 0x16); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
		Delay_us(DELAY);
		
		//printf("AK8963_WIA: %d \n",MPU92_Mag_ReadReg(AK8963_WIA));
		
		return 0;
}	

//====SPIд�Ĵ���
//====reg:ָ���ļĴ�����ַ
//====value:д���ֵ
uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	MPU9250_CS=0;  //ʹ��SPI����
	status=Spi_RW(reg); //����д����+�Ĵ�����
	Spi_RW(value);//д��Ĵ���ֵ
	MPU9250_CS=1;  //��ֹMPU9250
	return(status);//����״ֵ̬
}

//====SPI��ȡ�Ĵ���
//====reg:ָ���ļĴ�����ַ
uint8_t MPU9250_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	MPU9250_CS=0;  //ʹ��SPI����
	Spi_RW(reg|0x80); //====���Ͷ�����+�Ĵ�����
	reg_val=Spi_RW(0xff);//====��ȡ�Ĵ���ֵ
	MPU9250_CS=1;  //��ֹMPU9250
	return(reg_val);
}
//====SPIд�ŸмƼĴ���
//====writeAddr:ָ���ļĴ�����ַ
//====writeData:д���ֵ
void MPU92_Mag_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
	uint8_t  status = 0;
	MPU9250_Write_Reg(I2C_SLV4_ADDR ,AK8963_ADDR);//���ô����Ƶ�ַ,mode: writ
	Delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_REG ,writeAddr);//set reg addr
	Delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_DO ,writeData);//send value	
	Delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_CTRL ,0x81);
	do {
    status = MPU9250_Read_Reg(I2C_MST_STATUS);
    Delay_us(DELAY);
  } while (((status & I2C_SLV4_DONE) == 0));
}
//====SPI���ŸмƼĴ���
//====readAddr:ָ���ļĴ�����ַ
uint8_t MPU92_Mag_ReadReg( uint8_t readAddr )
{
	u16 j=500;
	MPU9250_Write_Reg(I2C_SLV4_ADDR ,AK8963_ADDR|0x80); //���ô����Ƶ�ַ��mode��read
	Delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_REG ,readAddr);// set reg addr
	Delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_CTRL ,0x80);
	Delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_DO ,0xff);//read
	Delay_us(1000);
	while(j--);//�˴���ΪMPU�ڲ�I2C��ȡ�ٶȽ�����������ʱ�ȴ��ڲ���ȡ���
	return MPU9250_Read_Reg(I2C_SLV4_DI);
}
//====��ȡMPU9250����
void MPU9250_ReadValue(void)
{
	uint8_t number;
	uint8_t MPU9250_buf[22],i;
	int16_t MatrixAccelData[3];
	int16_t	translateAccelData[3];
	
	int16_t MatrixGyroData[3];
	int16_t translateGyroData[3];
	
	int16_t MatrixMagData[3];
	int16_t translateMagData[3];
	
	MPU9250_CS=0;  //====ʹ��SPI����
	Delay_us(3);
	Spi_RW(ACCEL_XOUT_H|0x80); //====���Ͷ�����+�Ĵ�����
	for(i=0;i<22;i++)//====һ����ȡ14�ֽڵ�����
	{
		MPU9250_buf[i]=Spi_RW(0xff); //====ѭ����ȡ
	}
	MPU9250_CS=1;  //====��ֹMPU9250
	Delay_us(1);
	
	
    rawAccel[YORIENT].bytes[1]       = MPU9250_buf[ 0];//====����ԭʼ����
    rawAccel[YORIENT].bytes[0]       = MPU9250_buf[ 1];
    rawAccel[XORIENT].bytes[1]       = MPU9250_buf[ 2];
    rawAccel[XORIENT].bytes[0]       = MPU9250_buf[ 3];
    rawAccel[ZORIENT].bytes[1]       = MPU9250_buf[ 4];
    rawAccel[ZORIENT].bytes[0]       = MPU9250_buf[ 5];

    rawMPU6050Temperature.bytes[1] = MPU9250_buf[ 6];
    rawMPU6050Temperature.bytes[0] = MPU9250_buf[ 7];

    rawGyro[PITCH].bytes[1]        = MPU9250_buf[ 8];
    rawGyro[PITCH].bytes[0]        = MPU9250_buf[ 9];
    rawGyro[ROLL ].bytes[1]        = MPU9250_buf[10];
    rawGyro[ROLL ].bytes[0]        = MPU9250_buf[11];
    rawGyro[YAW  ].bytes[1]        = MPU9250_buf[12];
    rawGyro[YAW  ].bytes[0]        = MPU9250_buf[13];
	
		rawMag[YORIENT].bytes[1]       = MPU9250_buf[15];
		rawMag[YORIENT].bytes[0]       = MPU9250_buf[16];
		rawMag[XORIENT].bytes[1]       = MPU9250_buf[17];
		rawMag[XORIENT].bytes[0]       = MPU9250_buf[18];
		rawMag[ZORIENT].bytes[1]       = MPU9250_buf[19];
		rawMag[ZORIENT].bytes[0]       = MPU9250_buf[20];
	
//	for (number = 0; number < 3; number++)
//   {
//        MatrixAccelData[number] = rawAccel[number].value;		//====��ȡ���ٶȾ���B��ֵ
//        MatrixGyroData[number]  = rawGyro[number].value;		//====��ȡ�����Ǿ���B��ֵ
//				MatrixMagData[number]		= rawMag[number].value;			//====��ȡ�Ÿмƾ���B��ֵ
//   }
	/* �÷�����������ǰ����λ -------------------------------------------------*/	
//	matrixMultiply(3, 3, 1, translateAccelData, orientationMatrix, MatrixAccelData);	
//	matrixMultiply(3, 3, 1, translateGyroData,  orientationMatrix, MatrixGyroData);
//	matrixMultiply(3, 3, 1, translateMagData,  orientationMatrix, MatrixMagData);
//	 
//	for (number = 0; number < 3; number++)
//  {
//    rawAccel[number].value = translateAccelData[number];//���ݾ�����˺�Ľ��
//    rawGyro[number].value  = translateGyroData[number];
//		rawMag[number].value   = translateMagData[number];
//  }
}

//void orientIMU(void) //����IMU��Ԫ�ķ�λȷ������A��ֵ 
//{
//    switch (SDCardConfig.imuOrientation)
//    {
//        case 1: // Dot Front/Left/Top
//            orientationMatrix[0] =  1;
//            orientationMatrix[1] =  0;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  0;
//            orientationMatrix[4] =  1;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] =  1;
//            break;

//        case 2: // Dot Front/Right/Top
//            orientationMatrix[0] =  0;
//            orientationMatrix[1] = -1;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  1;
//            orientationMatrix[4] =  0;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] =  1;
//            break;

//        case 3: // Dot Back/Right/Top
//            orientationMatrix[0] = -1;
//            orientationMatrix[1] =  0;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  0;
//            orientationMatrix[4] = -1;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] =  1;
//            break;

//        case 4: // Dot Back/Left/Top
//            orientationMatrix[0] =  0;
//            orientationMatrix[1] =  1;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] = -1;
//            orientationMatrix[4] =  0;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] =  1;
//            break;

//        case 5: // Dot Front/Left/Bottom
//            orientationMatrix[0] =  0;
//            orientationMatrix[1] = -1;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] = -1;
//            orientationMatrix[4] =  0;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] = -1;
//            break;

//        case 6: // Dot Front/Right/Bottom
//            orientationMatrix[0] =  1;
//            orientationMatrix[1] =  0;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  0;
//            orientationMatrix[4] = -1;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] = -1;
//            break;

//        case 7: // Dot Back/Right/Bottom
//            orientationMatrix[0] =  0;
//            orientationMatrix[1] =  1;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  1;
//            orientationMatrix[4] =  0;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] = -1;
//            break;

//        case 8: // Dot Back/Left/Bottom
//            orientationMatrix[0] = -1;
//            orientationMatrix[1] =  0;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  0;
//            orientationMatrix[4] =  1;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] = -1;
//            break;

//				case 9: // Dot
//            orientationMatrix[0] =  0;
//            orientationMatrix[1] =  -1;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  -1;
//            orientationMatrix[4] =  0;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] =  1;
//            break;
//				
//        default: // Dot Front/Left/Top
//            orientationMatrix[0] =  1;
//            orientationMatrix[1] =  0;
//            orientationMatrix[2] =  0;
//            orientationMatrix[3] =  0;
//            orientationMatrix[4] =  1;
//            orientationMatrix[5] =  0;
//            orientationMatrix[6] =  0;
//            orientationMatrix[7] =  0;
//            orientationMatrix[8] =  1;
//            break;
//    }
//}

#endif


