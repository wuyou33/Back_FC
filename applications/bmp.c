#include "include.h"
#include "bmp.h"
#include "iic_hml.h"
#include "imu.h"
#include "height_ctrl.h"
#include <math.h>
#include "rc.h"
#include "time.h"
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;
float baro_alt_speed_ano;
int baroAlt,baroAlt_fc;
u8 baro_set;
_height_st baro;

#undef ALTI_SPEED

//#define MS5611Press_OSR  MS561101BA_OSR_2048  //��ѹ��������
//#define MS5611Temp_OSR   MS561101BA_OSR_2048 //�¶Ȳ�������
#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096 //�¶Ȳ�������
// ��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ �¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	  //��ʼת�� ��ѹ
#define SCPressureing  0x04	  //����ת����ѹֵ

#define MOVAVG_SIZE  100//10	   //��ѹ���˲�����

static uint8_t  Now_doing = SCTemperature;	//��ǰת��״̬
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //�궨ֵ���
static uint32_t Current_delay=0;	    //ת����ʱʱ�� us 
static uint32_t Start_Convert_Time; //����ת��ʱ�� ʱ�� us 
static int32_t  tempCache;

static float Alt_Offset_m = 0;

//
#define PA_OFFSET_INIT_NUM 50	

static float Alt_offset_Pa=99180; //�����0��(���������ƽ��)ʱ ��Ӧ����ѹֵ  ���ֵ����ϵ�ʱ����ѹֵ 
double paOffsetNum = 0; 
uint16_t  paInitCnt=0;
uint8_t paOffsetInited=0;

//interface for outside 
uint8_t Baro_ALT_Updated = 0; //��ѹ�Ƹ߶ȸ�����ɱ�־��
//units (Celsius degrees*100, mbar*100  ).
//��λ [�¶� ��] [��ѹ ��]  [�߶� ��] 
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;

// ��ʱ��λ us 	  ��ͬ�Ĳ������ȶ�Ӧ��ͬ����ʱֵ
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	//11000,//MS561101BA_OSR_4096 9.1ms 0x08
	20000, // 16.44ms
};

// FIFO ����					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //����ָ��

//���һ���µ�ֵ�� �¶ȶ��� �����˲�
void MS561101BA_NewTemp(float val) 
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}

//���һ���µ�ֵ�� ��ѹ���� �����˲�
void MS561101BA_NewPress(float val)
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}

//���һ���µ�ֵ�� �߶ȶ��� �����˲�
void MS561101BA_NewAlt(float val) 
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	Alt_buffer[MOVAVG_SIZE-1] = val;
}

//��ȡ���е�ƽ��ֵ
float MS561101BA_getAvg(float * buff, int size) 
{
	float sum = 0.0;
	int i;
	for(i=0; i<size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_readPROM(void)
*��������:	    ��ȡ MS561101B �Ĺ����궨ֵ
��ȡ ��ѹ�Ƶı궨ֵ  ���������¶Ⱥ���ѹ�Ķ���
*******************************************************************************/
void MS561101BA_readPROM(void) 
{
	u8  inth,intl;
	uint8_t i2cret[2];
	int i;
	for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) 
	{
		#ifdef DEBUG_HW_I2C
			i2cRead(MS5611_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2, i2cret); 
			PROM_C[i]=i2cret[0]<<8 | i2cret[1];
		#else
		
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR);
			IIC_Wait_Ack();
			IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
			IIC_Wait_Ack();	
			IIC_Stop();
			Delay_us(5);
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
			Delay_us(1);
			IIC_Wait_Ack();
			inth = IIC_Read_Byte(1);  //��ACK�Ķ�����
			Delay_us(1);
			intl = IIC_Read_Byte(0);	 //���һ���ֽ�NACK
			IIC_Stop();
//			IIC_Read_nByte(MS5611_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2, i2cret); 
			PROM_C[i] = (((uint16_t)inth << 8) | intl);
		#endif
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_reset(void)
*��������:	    ���͸�λ��� MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) 
{
	#ifdef DEBUG_HW_I2C
		i2cWrite(MS5611_ADDR, MS561101BA_RESET, 1); 
	#else
	IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
    IIC_Send_Byte(MS561101BA_RESET);//���͸�λ����
	IIC_Wait_Ack();	
    IIC_Stop();
//	IIC_Write_1Byte(MS5611_ADDR,MS561101BA_RESET,1);
	#endif
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_startConversion(uint8_t command)
*��������:	    ��������ת����� MS561101B
��ѡ�� ת������Ϊ MS561101BA_D1  ת����ѹ
				  MS561101BA_D2  ת���¶�	 
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command) 
{
#ifdef DEBUG_HW_I2C
	i2cWrite(MS5611_ADDR, command, 1); 
#else
	// initialize pressure conversion
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //дת������
	IIC_Wait_Ack();	
	IIC_Stop();
//	IIC_Write_1Byte(MS5611_ADDR,command,1);
#endif
}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned long MS561101BA_getConversion(void)
*��������:	    ��ȡ MS561101B ��ת�����	 
*******************************************************************************/
uint32_t MS561101BA_getConversion(void) 
{
	uint32_t conversion = 0;
	u8 temp[3];
	#ifdef DEBUG_HW_I2C
		i2cRead(MS5611_ADDR,CMD_ADC_READ ,3, temp); 
		conversion=temp[0] << 16 | temp[0] <<8 | temp[2];
	#else
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(0);// start read sequence
	IIC_Wait_Ack();	
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
	IIC_Wait_Ack();
	temp[0] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
	temp[1] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
	temp[2] = IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
	IIC_Stop();
	//IIC_Read_nByte(MS5611_ADDR,CMD_ADC_READ ,3, temp); 
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
	#endif
	return conversion;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_init(void)
*��������:	    ��ʼ�� MS561101B 
*******************************************************************************/
void MS5611_Init_FC(void) 
{  
	MS561101BA_reset(); // ��λ MS561101B 
	Delay_ms(100); // ��ʱ 
	MS561101BA_readPROM(); // ��ȡEEPROM �еı궨ֵ ����	

	MS5611_Init();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_GetTemperature(void)
*��������:	    ��ȡ �¶�ת�����	 
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{	
	tempCache = MS561101BA_getConversion();	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		float MS561101BA_get_altitude(void)
*��������:	    ����ǰ����ѹֵת�� �߶ȡ�	 
*******************************************************************************/
u8 for_fly_ready;
float MS561101BA_get_altitude(void)
{
	static float Altitude,AltPre;
	float dz,dt;
	uint32_t current=0;
	static uint32_t tp=0;

//	// �Ƿ��ʼ����0����ѹֵ��
//	if(Alt_offset_Pa == 0)
//	{ 
//		if(paInitCnt > PA_OFFSET_INIT_NUM)
//		{
//			Alt_offset_Pa = paOffsetNum / paInitCnt;
//			paOffsetInited=1;
//		}
//		else
//			paOffsetNum += MS5611_Pressure;
//		
//		paInitCnt++;
//		
//		Altitude = 0; //�߶� Ϊ 0
//		
//		return Altitude;
//	}
	//����������ϵ�ʱ��λ�õĸ߶�ֵ ����λΪm
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903))*0.01f;  
	#if !DEBUG_WITHOUT_SB
		if(!fly_ready&&NS==2)		
		Alt_Offset_m=Altitude;
	#endif
	Altitude = Altitude - Alt_Offset_m ;  //��ƫ��

	#ifdef ALTI_SPEED
	current=GetSysTime_us;//micros();
	dt=(tp>0)?((current - tp)/1000000.0f):0;
	tp=current;
	dz=(Altitude-AltPre);
	AltPre=Altitude;	//m
	if(dt>0)
		MS5611_VerticalSpeed =  dz / dt;
#endif
	
	return Altitude; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_getPressure(void)
*��������:	    ��ȡ ��ѹת����� ������������	 
*******************************************************************************/

int baroAlt;
u8 baro_set;
float angle_baro=12;
void MS561101BA_getPressure(void) 
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS561101BA_getConversion();
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);
	
	TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;
	off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);
	
	if (TEMP < 2000)
	{   // second order temperature compensation
		T2 = (((int64_t)dT)*dT) >> 31;
		Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64)>>1;
		SENS2 = (5*Aux_64)>>2;
		TEMP = TEMP - T2;
		off = off - OFF2;
		sens = sens - SENS2;
	}
	float newPress;
  static float lastPress;
	//------------- ��ѹ���� ----------------
	newPress=(((((int64_t)rawPress) * sens) >> 21) - off) / 32768;
	
	float press_limit_coe = 1.0f; 
	
	//����ʱ������ѹֵ���ͣ���ѹ�߶����ߣ�
	if(mode_oldx.baro_lock&& (fabs(Pit_fc)>angle_baro || fabs(Rol_fc)>angle_baro)&&0)
	{		
		press_limit_coe = 0.01f;   //0.005
		if(newPress<lastPress)
			newPress = (1 - press_limit_coe) * lastPress + press_limit_coe * newPress; 
	}

	lastPress = newPress;
	
	MS5611_Pressure = newPress;
	
	//�¶ȶ��д���
	MS561101BA_NewTemp(TEMP*0.01f);
	
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
	MS5611_Altitude = MS561101BA_get_altitude(); // ��λ��m 
	baroAlt_fc=MS5611_Altitude*1000;
	static u8 cnt;
	if(cnt++>100){cnt=100;
		baro_set=1;}
}

void MS5611_ThreadNew(void) 
{
	switch(Now_doing)
	{ //��ѯ״̬ ������������ ����Щʲô��
 		case SCTemperature:  //�����¶�ת��
			//�����¶�ת��
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time =GetSysTime_us();// micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
 		break;
		
		case CTemperatureing:  //����ת���� 
			if((GetSysTime_us()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_GetTemperature(); //ȡ�¶�	
				//������ѹת��
				MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
				Current_delay = MS5611_Delay_us[MS5611Press_OSR];//ת��ʱ��
				Start_Convert_Time = GetSysTime_us();//��ʱ��ʼ
				Now_doing = SCPressureing;//��һ��״̬
			}
			break;
 
		case SCPressureing:	 //����ת����ѹֵ
			if((GetSysTime_us()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_getPressure();   //���� ����	
				Baro_ALT_Updated = 0xff; 	//�߶ȸ��� ��ɡ�
			//	Now_doing = SCTemperature;  //��ͷ����
				//�����¶�ת��
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = GetSysTime_us(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}
//ע�⣬ʹ��ǰȷ��
uint8_t  WaitBaroInitOffset(void)
{
	uint32_t startTime=0;
	uint32_t now=0;
	
	startTime=GetSysTime_us();	//us
  while(!paOffsetInited)
	{
			MS5611_ThreadNew();
			now=GetSysTime_us();
			if((now-startTime)/1000 >= PA_OFFSET_INIT_NUM * 50)	//��ʱ
			{
				return 0;
			}
	}
	
	return 1;
}


//ANO

#include "ms5611.h"
#include "filter.h"
#define BARO_CAL_CNT 80


	s32 baro_height_old;
	
	s32 baro_Offset;
	
	uint32_t ms5611_ut;  // static result of temperature measurement
	uint32_t ms5611_up;  // static result of pressure measurement
	uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
	uint8_t t_rxbuf[3],p_rxbuf[3];

void MS5611_Reset(void)
{
    IIC_Write_1Byte(MS5611_ADDR1, CMD_RESET, 1);
}

u8 MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		check += IIC_Read_nByte(MS5611_ADDR1, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if(check==PROM_NB)
		return 1;
	else
		return 0;
}


void MS5611_Read_Adc_T(void)
{
	IIC_Read_nByte( MS5611_ADDR1, CMD_ADC_READ, 3, t_rxbuf ); // read ADC
}

void MS5611_Read_Adc_P(void)
{
	IIC_Read_nByte(MS5611_ADDR1, CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void MS5611_Start_T(void)
{
	IIC_Write_1Byte(MS5611_ADDR1, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void MS5611_Start_P(void)
{
  IIC_Write_1Byte(MS5611_ADDR1, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}

u8 ms5611_ok;
void MS5611_Init(void)
{
	
	Delay_ms(10);
	//?????
	MS5611_Reset();
	Delay_ms(3);
	ms5611_ok = !( MS5611_Read_Prom() );
	//??????
	MS5611_Start_T();
}

int MS5611_Update(void)
{
	static int state = 0;
	
	//I2C_FastMode = 0;
	
	if (state) 
	{
			MS5611_Read_Adc_P();
			MS5611_Start_T();
			MS5611_BaroAltCalculate();
			state = 0;
	} 
	else 
	{
			MS5611_Read_Adc_T();
			MS5611_Start_P();
			state = 1;
	}
	return (state);
}

float temperature_5611;

_height_st baro;

//Moving_Median(float moavarray[],u16 len ,u16 *fil_p,float in)

#define MO_LEN 5
	s32 mo_av_baro[MO_LEN];
	u16 moavcnt;

void MS5611_BaroAltCalculate(void)
{
	static u8 baro_start;
	
  int32_t temperature, off2 = 0, sens2 = 0, delt;
  int32_t pressure;
	float alt_3;
	
	int32_t dT;
	int64_t off;
	int64_t sens;
	
		static vs32 sum_tmp_5611 = 0;
		static u8 sum_cnt = BARO_CAL_CNT + 10;
	
	
		ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
		ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
		
    dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
    off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
    sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
		//pressure = (int)((1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
		
		alt_3 = (101000 - pressure)/1000.0f;
		pressure = 0.82f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
							// - ( temperature_5611  ) *650 *(pressure/101000.0);
		
// 		if( pressure < 101000 )
// 		{
// 			pressure = 80 *my_pow( (101000 - pressure)/1000.0 ) + 0.08 *(101000 - pressure)*100.0;
// 			
// 		}
// 		else
// 		{
// 			pressure = baro_Offset;
// 		}
		
		//if(!(baro_Offset == 0))

				
			
			
			baro.height = (s32)( pressure  );//( my_deathzoom(( pressure - baro_Offset ), baroAlt ,baro_fix ) ) ; //cm + 
			//baro.relative_height = (pressure - baro_Offset);
			//baro_alt_speed += 10 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed ); // 20ms ?? /0.02 = *50 ??cm/s
			
			baro.relative_height =(s32)(pressure - baro_Offset)*10;;// Moving_Median(mo_av_baro,MO_LEN ,&moavcnt,(s32)(pressure - baro_Offset))*10;
	///////////////////		
			baro.h_delta = ( baro.height - baro_height_old )*10 ; 
			//LPF_1_(1.0f,0.02f,( baro.height - baro_height_old ),baro.h_delta);
			
		  baro_height_old = baro.height;
			
			baro.measure_ok = 1;
			
			if( baro_start < 50 )
			{
				baro_start++;
				baro.h_delta = 0;

				baro.relative_height = 0;
				
				if(baro_start<10)
				{
					baro_Offset = pressure;
				}
				else
				{
					baro_Offset += 10.0f *3.14f *0.02f *(pressure - baro_Offset);
				}
			}	
			
		if(sum_cnt)
		{
			sum_cnt--;
// 			if(sum_cnt < BARO_CAL_CNT)
// 				sum_tmp_5611 += pressure;
// 			if(sum_cnt==0)
// 				baro_Offset = sum_tmp_5611 / (BARO_CAL_CNT - 1);
		}
		else
		{
			temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
		}
			
		
}



