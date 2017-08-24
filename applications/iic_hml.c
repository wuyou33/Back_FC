
#include "iic_hml.h"

  void I2c_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(1)
	{
		u8 i = 15;
		while(i--);
	}
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
void IIC_Init(void)
{			
 GPIO_InitTypeDef  GPIO_InitStructure;
  #if USE_ZIN_BMP
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	IIC_SCL_T=1;
	IIC_SDA_T=1;
	#else
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
	
	#if USE_MINI_FC_FLOW_BOARD
	IIC_SCL_F=1;
	IIC_SDA_F=1;
	#else
	IIC_SCL=1;
	IIC_SDA=1;
	#endif
	#endif
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void IIC_Start(void)
{
  #if USE_ZIN_BMP
	SDA_OUT_T();     //sda�����
	IIC_SDA_T=1;	  	  
	IIC_SCL_T=1;
	I2c_delay();
 	IIC_SDA_T=0;//START:when CLK is high,DATA change form high to low 
	I2c_delay();
	IIC_SCL_T=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
#else	
#if USE_MINI_FC_FLOW_BOARD
	SDA_OUT_F();     //sda�����
	IIC_SDA_F=1;	  	  
	IIC_SCL_F=1;
	I2c_delay();
 	IIC_SDA_F=0;//START:when CLK is high,DATA change form high to low 
	I2c_delay();
	IIC_SCL_F=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
#else	
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	I2c_delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	I2c_delay();
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
#endif
#endif	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
  #if USE_ZIN_BMP
	SDA_OUT_T();//sda�����
	IIC_SCL_T=0;
	IIC_SDA_T=0;//STOP:when CLK is high DATA change form low to high
 	I2c_delay();
	IIC_SCL_T=1; 
	IIC_SDA_T=1;//����I2C���߽����ź�
	I2c_delay();	
#else	
#if USE_MINI_FC_FLOW_BOARD
	SDA_OUT_F();//sda�����
	IIC_SCL_F=0;
	IIC_SDA_F=0;//STOP:when CLK is high DATA change form low to high
 	I2c_delay();
	IIC_SCL_F=1; 
	IIC_SDA_F=1;//����I2C���߽����ź�
	I2c_delay();		
#else	
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	I2c_delay();
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	I2c_delay();		
#endif	
#endif	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
  #if USE_ZIN_BMP
u8 ucErrTime=0;
	SDA_IN_T();      //SDA����Ϊ����  
	IIC_SDA_T=1;Delay_us(1);	   
	IIC_SCL_T=1;Delay_us(1);	 
	while(READ_SDA_T)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  Delay_us(1);
	}
	IIC_SCL_T=0;//ʱ�����0 	   
	return 0;  

#else	
#if USE_MINI_FC_FLOW_BOARD
	u8 ucErrTime=0;
	SDA_IN_F();      //SDA����Ϊ����  
	IIC_SDA_F=1;Delay_us(1);	   
	IIC_SCL_F=1;Delay_us(1);	 
	while(READ_SDA_F)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  Delay_us(1);
	}
	IIC_SCL_F=0;//ʱ�����0 	   
	return 0;  
#else	
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;Delay_us(1);	   
	IIC_SCL=1;Delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  Delay_us(1);
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
#endif	
#endif
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
  #if USE_ZIN_BMP
	IIC_SCL_T=0;
	SDA_OUT_T();
	IIC_SDA_T=0;
	Delay_us(2);
	IIC_SCL_T=1;
	Delay_us(2);
	IIC_SCL_T=0;
#else	
#if USE_MINI_FC_FLOW_BOARD
	IIC_SCL_F=0;
	SDA_OUT_F();
	IIC_SDA_F=0;
	Delay_us(2);
	IIC_SCL_F=1;
	Delay_us(2);
	IIC_SCL_F=0;
#else	
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
#endif	
#endif	
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
 #if USE_ZIN_BMP	
		IIC_SCL_T=0;
	SDA_OUT_T();
	IIC_SDA_T=1;
	Delay_us(2);
	IIC_SCL_T=1;
	Delay_us(2);
	IIC_SCL_T=0;
	#else
#if USE_MINI_FC_FLOW_BOARD
	IIC_SCL_F=0;
	SDA_OUT_F();
	IIC_SDA_F=1;
	Delay_us(2);
	IIC_SCL_F=1;
	Delay_us(2);
	IIC_SCL_F=0;
#else	
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
#endif	
	#endif
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{ 
 #if USE_ZIN_BMP	
    u8 t;   
	SDA_OUT_T(); 	    
    IIC_SCL_T=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA_T=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL_T=1;
		Delay_us(2); 
		IIC_SCL_T=0;	
		Delay_us(2);
    }	
#else	
#if USE_MINI_FC_FLOW_BOARD
    u8 t;   
	SDA_OUT_F(); 	    
    IIC_SCL_F=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA_F=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL_F=1;
		Delay_us(2); 
		IIC_SCL_F=0;	
		Delay_us(2);
    }	
#else	
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL=1;
		Delay_us(2); 
		IIC_SCL=0;	
		Delay_us(2);
    }	
#endif	
#endif		
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
 #if USE_ZIN_BMP	
	unsigned char i,receive=0;
	SDA_IN_T();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_T=0; 
        Delay_us(2);
		IIC_SCL_T=1;
        receive<<=1;
        if(READ_SDA_T)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
#else	
#if USE_MINI_FC_FLOW_BOARD
	unsigned char i,receive=0;
	SDA_IN_F();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_F=0; 
        Delay_us(2);
		IIC_SCL_F=1;
        receive<<=1;
        if(READ_SDA_F)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
#else	
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
#endif	
#endif		
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBitsm(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBitm(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}


#define ZIN_35_ADDRESS 0xB0 //ģ���ַ ���λ��д��־λ 0��д  1����

//����������������ģ�� ADDR��д��+ CMD����
#define MODULE_REST 0x20 	//������Ҫ�� ���Բ���
#define MODULE_GET_OFFSET 0x21  //������У׼����
#define READ_FLASH 0x22  //ʹģ���ڲ���ȡ У׼����

#define EKF_FILTER_ON 0x24
#define EKF_FILTER_OFF 0x25

//��������������ȡģ������  ADDR��д��+REG+ADDR���� = д+1�� + ......���ݴ���֧��������ַ�ϵ�����������ȡ
#define MPU_ACC_READ 0x30
#define MPU_GYRO_READ 0x31
#define ANGLE_READ 0x32
#define HEIGHT_READ 0X33 //�߶����ݼĴ�����ַ
#define RATE_READ 0X34 //
float rate,height;
struct _st_angle
{
	float roll;
	float pitch;
	float yaw;
}angle;



/******************************************************************************
 * ��������: I2c_Start
 * ��������: I2c  ��ʼ�ź�
 * ��ڲ���: ��
 ******************************************************************************/
static uint8_t I2c_Start(void)
{
    SDA_H;
    SCL_H;
	I2c_delay();
    if (!SDA_read)
        return 0;
    SDA_L;
    I2c_delay();
    if (SDA_read)
        return 0;
    SCL_L;
    I2c_delay();
    return 1;
}

/******************************************************************************
 * ��������: I2c_Stop
 * ��������: I2c  ֹͣ�ź�
 * ��ڲ���: ��
 ******************************************************************************/
static void I2c_Stop(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
	I2c_delay();
    I2c_delay();
    SCL_H;
	I2c_delay();
    SDA_H;
    I2c_delay();
}

/******************************************************************************
 * ��������: I2c_Ack
 * ��������: I2c  ����Ӧ���ź�
 * ��ڲ���: ��
 ******************************************************************************/
static void I2c_Ack(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/******************************************************************************
 * ��������: I2c_NoAck
 * ��������: I2c  ����NAck
 * ��ڲ���: ��
 ******************************************************************************/
static void I2c_NoAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/*******************************************************************************
 *��������:	I2c_WaitAck
 *��������:	�ȴ�Ӧ���źŵ���
 *����ֵ��   1������Ӧ��ʧ��
 *           0������Ӧ��ɹ�
 *******************************************************************************/
static uint8_t I2c_WaitAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
    I2c_delay();
	
    if (SDA_read) {
        SCL_L;
        return 0;
    }
    SCL_L;
    return 1;
}

/******************************************************************************
 * ��������: I2c_SendByte
 * ��������: I2c  ����һ���ֽ�����
 * ��ڲ���: byte  ���͵�����
 ******************************************************************************/
static void I2c_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2c_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2c_delay();
        SCL_H;
		I2c_delay();
		I2c_delay();
		I2c_delay();
    }
    SCL_L;
}

/******************************************************************************
 * ��������: I2c_ReadByte
 * ��������: I2c  ��ȡһ���ֽ�����
 * ��ڲ���: ��
 * ����ֵ	 ��ȡ������
 ******************************************************************************/
static uint8_t I2c_ReadByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2c_delay();
		I2c_delay();
        SCL_H;
		I2c_delay();
        I2c_delay();
		I2c_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

/******************************************************************************
 * ��������: i2cWriteBuffer
 * ��������: I2c       ���豸��ĳһ����ַд��̶����ȵ�����
 * ��ڲ���: addr,     �豸��ַ
 *           reg��     �Ĵ�����ַ
 *			 len��     ���ݳ���
 *			 *data	   ����ָ��
 * ����ֵ	 1
 ******************************************************************************/
uint8_t i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2c_Start())
        return 0;
    I2c_SendByte(addr);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    for (i = 0; i < len; i++) {
        I2c_SendByte(data[i]);
        if (!I2c_WaitAck()) {
            I2c_Stop();
            return 0;
        }
    }
    I2c_Stop();
    return 1;
}
/////////////////////////////////////////////////////////////////////////////////
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		return 1;
	}
	else
	{
		return 0;
	}
	//return 0;
}

/******************************************************************************
 * ��������: i2cread
 * ��������: I2c  ���豸��ĳһ����ַ��ȡ�̶����ȵ�����
 * ��ڲ���: addr,   �豸��ַ
 *           reg��   �Ĵ�����ַ�׵�ַ
 *			 len��   ���ݳ���
 *			 *buf	 ����ָ��
 * ����ֵ	 �ɹ� ���� 1
 *           ���� ���� 0
 ******************************************************************************/


/*****************************************************************************
 *��������:	i2cWrite
 *��������:	д��ָ���豸 ָ���Ĵ���һ���ֽ�
 *��ڲ����� addr Ŀ���豸��ַ
 *		     reg   �Ĵ�����ַ
 *		     data ���������ݽ�Ҫ��ŵĵ�ַ
 *******************************************************************************/
uint8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2c_Start())
        return 0;
    I2c_SendByte(addr);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    I2c_SendByte(data);
    I2c_WaitAck();
    I2c_Stop();
    return 1;
}


uint8_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2c_Start())
        return 0;
    I2c_SendByte(addr);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
	I2c_Stop();
    I2c_Start();
    I2c_SendByte(addr+1);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    while (len) {
        *buf = I2c_ReadByte();
        if (len == 1)
            I2c_NoAck();
        else
            I2c_Ack();
        buf++;
        len--;
    }
    I2c_Stop();
    return 1;
}


int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(i2cRead(addr,reg,len,buf))
	{
		return 1;
	}
	else
	{
		return 0;
	}
	//return 0;
}

  float ByteToFloat( u8* byteArry)
{
  return *((float*)byteArry);
}
float z_speed123;
u8 buf_test[4];
void read_zin(void){
	  //i2cRead(ZIN_35_ADDRESS,MPU_ACC_READ,12,(uint8_t *)&mpu.accX);    //�ó�MPU��ԭʼֵ�������˲������ֵ����ÿ�����ֽ�Ϊһ�����ݣ���6��short int�����ݣ�
		
		//IICreadBytes(ZIN_35_ADDRESS,ANGLE_READ,12,(uint8_t *)&angle);       //�ó��Ƕ�ֵ��ÿ�ĸ��ֽ�Ϊһ�����ݣ���3��float������
		IICreadBytes(ZIN_35_ADDRESS,RATE_READ,4,(uint8_t *)&z_speed123);         //�ó���ֱ�����ϵ��ٶȣ��ĸ��ֽ�Ϊһ��float����
   // z_speed123=ByteToFloat(buf_test);
		//IICreadBytes(ZIN_35_ADDRESS,HEIGHT_READ,4,(uint8_t *)&height);     //�ó��߶�ֵ���ĸ��ֽ�Ϊһ��float����
}
	//------------------End of File----------------------------
