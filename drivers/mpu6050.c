#include "mpu6050.h"
#include "mymath.h"
#include "i2c_soft.h"
#include "imu.h"
MPU6050_STRUCT mpu6050,mpu6050_fc;

u8 mpu6050_buffer[14];
u8 mpu6050_ok;
void MPU6050_Read(void)
{
    I2C_FastMode = 1;
    IIC_Read_nByte(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
失败为0
*******************************************************************************/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data) {
    u8 b;
    IIC_Read_nByte(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_ok = !( IIC_Write_1Byte(dev, reg, b) );
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1
失败为0
*******************************************************************************/
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b,mask;
    IIC_Read_nByte(dev, reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    IIC_Write_1Byte(dev, reg, b);
}

/**************************实现函数********************************************
*函数原型:
*功　　能:	    设置 采样率
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
    IIC_Write_1Byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
    //I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //不自检
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //不自检
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/**************************实现函数********************************************
*函数原型:
*功　　能:	    设置低通滤波截止频率
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_INT_Config()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_SetBits(GPIOD, GPIO_Pin_7);

}
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(u16 lpf)
{
    u16 default_filter = 1;

    MPU6050_INT_Config();

    switch(lpf)
    {
    case 5:
        default_filter = MPU6050_DLPF_BW_5;
        break;
    case 10:
        default_filter = MPU6050_DLPF_BW_10;
        break;
    case 20:
        default_filter = MPU6050_DLPF_BW_20;
        break;
    case 42:
        default_filter = MPU6050_DLPF_BW_42;
        break;
    case 98:
        default_filter = MPU6050_DLPF_BW_98;
        break;
    case 188:
        default_filter = MPU6050_DLPF_BW_188;
        break;
    case 256:
        default_filter = MPU6050_DLPF_BW_256;
        break;
    default:
        default_filter = MPU6050_DLPF_BW_42;
        break;
    }
    I2c_Soft_Init();

    //设备复位
//	IIC_Write_1Byte(MPU6050_ADDR,MPU6050_RA_PWR_MGMT_1, 0x80);

    MPU6050_setSleepEnabled(0); //进入工作状态
    Delay_ms(10);
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //设置时钟  0x6b   0x03
    Delay_ms(10);
    MPU6050_set_SMPLRT_DIV(1000);  //1000hz
    Delay_ms(10);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
    Delay_ms(10);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//加速度度最大量程 +-8G
    Delay_ms(10);
    MPU6050_setDLPF(default_filter);  //42hz
    Delay_ms(10);
    MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
    Delay_ms(10);
    MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
    Delay_ms(10);	
}
#include "cycle_cal_oldx.h"
s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
#define OFFSET_AV_NUM_ACC 50
void MPU6050_Data_Offset()
{
#ifdef ACC_ADJ_EN

    if(mpu6050_fc.Acc_CALIBRATE == 1||need_init_mems==1)
    {
        if(my_sqrt(my_pow(mpu6050_fc.Acc_I16.x)+my_pow(mpu6050_fc.Acc_I16.y)+my_pow(mpu6050_fc.Acc_I16.z)) < 2500)
        {
            sensor_setup.Offset.mpu_flag = 1;
        }
        else if(my_sqrt(my_pow(mpu6050_fc.Acc_I16.x)+my_pow(mpu6050_fc.Acc_I16.y)+my_pow(mpu6050_fc.Acc_I16.z)) > 2600)
        {
            sensor_setup.Offset.mpu_flag = 0;
        }

        acc_sum_cnt++;
				if(mpu6050_fc.Cali_3d){
//			  sum_temp[A_X] += (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x ;
//        sum_temp[A_Y] += (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y ;
//        sum_temp[A_Z] += (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z - 65536/16;
				  sum_temp_att[0]+=Pit_fc1;
					sum_temp_att[1]+=Rol_fc1;
				}
				
				{
        sum_temp[A_X] += mpu6050_fc.Acc_I16.x;
        sum_temp[A_Y] += mpu6050_fc.Acc_I16.y;
        sum_temp[A_Z] += mpu6050_fc.Acc_I16.z - 65536/16;   // +-8G
				}
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {   
            mpu6050_fc.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[0]=(float)sum_temp_att[0]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[1]=(float)sum_temp_att[1]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            acc_sum_cnt =0;
            mpu6050_fc.Acc_CALIBRATE = 0;
            WRITE_PARM();
					  need_init_mems=2;
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
				  	sum_temp_att[1]=sum_temp_att[0]=0;
        }
    }
// 3d cal
		static xyz_f_t ACC_Reg;
		static u8 acc_3d_step_reg;
		float sphere_x,sphere_y,sphere_z,sphere_r;
		switch(acc_3d_step)
			{ 
			case 0:
				acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;
				cycle_init_oldx(&hml_lsq);
			break;
			default:
			if(hml_lsq.size<360&&(fabs(ACC_Reg.x-mpu6050_fc.Acc_I16.x)>0||fabs(ACC_Reg.y-mpu6050_fc.Acc_I16.y)>0||fabs(ACC_Reg.z-mpu6050_fc.Acc_I16.z)>0))
			{
			if(acc_3d_step>acc_3d_step_reg)
			acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;		
			acc_sum_cnt_3d++;
      sum_temp_3d[A_X] += mpu6050_fc.Acc_I16.x;
      sum_temp_3d[A_Y] += mpu6050_fc.Acc_I16.y;
      sum_temp_3d[A_Z] += mpu6050_fc.Acc_I16.z;   
				if(acc_sum_cnt_3d>OFFSET_AV_NUM_ACC){
					if(acc_smple_cnt_3d<12){
					acc_smple_cnt_3d++;	
					xyz_f_t data;	
					data.x = sum_temp_3d[A_X]/OFFSET_AV_NUM_ACC;
					data.y = sum_temp_3d[A_Y]/OFFSET_AV_NUM_ACC;
					data.z = sum_temp_3d[A_Z]/OFFSET_AV_NUM_ACC;	
					acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;	
					cycle_data_add_oldx(&hml_lsq, (float)data.x/1000.,(float)data.y/1000.,(float)data.z/1000.);}
					else if(acc_3d_step==6){
					acc_3d_step=0;	
					cycle_cal_oldx(&hml_lsq, 666,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);	
					mpu6050_fc.Off_3d.x=(hml_lsq.Off[0]*1000);
					mpu6050_fc.Off_3d.y=(hml_lsq.Off[1]*1000);
					mpu6050_fc.Off_3d.z=(hml_lsq.Off[2]*1000);
					mpu6050_fc.Gain_3d.x =  (hml_lsq.Gain[0]);
					mpu6050_fc.Gain_3d.y =  (hml_lsq.Gain[1]);
					mpu6050_fc.Gain_3d.z =  (hml_lsq.Gain[2]);	
          WRITE_PARM();						
					}		 		
				} 
			acc_3d_step_reg=acc_3d_step;	
			}
			break;
		
		}
		ACC_Reg.x=mpu6050_fc.Acc_I16.x;
	  ACC_Reg.y=mpu6050_fc.Acc_I16.y;
		ACC_Reg.z=mpu6050_fc.Acc_I16.z;
#endif

    if(mpu6050_fc.Gyro_CALIBRATE||need_init_mems==2)
    {
        gyro_sum_cnt++;
        sum_temp[G_X] += mpu6050_fc.Gyro_I16.x;
        sum_temp[G_Y] += mpu6050_fc.Gyro_I16.y;
        sum_temp[G_Z] += mpu6050_fc.Gyro_I16.z;
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
            mpu6050_fc.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            gyro_sum_cnt =0;need_init_mems=0;
            if(mpu6050_fc.Gyro_CALIBRATE == 1)
			{
               WRITE_PARM();
			}  
            mpu6050_fc.Gyro_CALIBRATE = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
        }
    }
}

void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
    *it_x = itx;
    *it_y = ity;
    *it_z = itz;

}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float mpu6050_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;

void MPU6050_Data_Prepare(float T)
{
    u8 i;
    s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
//	float auto_offset_temp[3];
    float Gyro_tmp[3];


    MPU6050_Data_Offset(); //校准函数

    /*读取buffer原始数据*/
    mpu6050_fc.Acc_I16.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
    mpu6050_fc.Acc_I16.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
    mpu6050_fc.Acc_I16.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;

    mpu6050_fc.Gyro_I16.x = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
    mpu6050_fc.Gyro_I16.y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
    mpu6050_fc.Gyro_I16.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;

    Gyro_tmp[0] = mpu6050_fc.Gyro_I16.x ;//
    Gyro_tmp[1] = mpu6050_fc.Gyro_I16.y ;//
    Gyro_tmp[2] = mpu6050_fc.Gyro_I16.z ;//

    mpu6050_fc.Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature
    mpu6050_fc.TEM_LPF += 2 *3.14f *T *(mpu6050_fc.Tempreature - mpu6050_fc.TEM_LPF);
    mpu6050_fc.Ftempreature = mpu6050_fc.TEM_LPF/340.0f + 36.5f;

//======================================================================
    if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
    {
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
    }
//10 170 4056
		if(fabs(mpu6050_fc.Off_3d.x)>10||fabs(mpu6050_fc.Off_3d.y)>10||fabs(mpu6050_fc.Off_3d.z)>10)
			mpu6050_fc.Cali_3d=1;
		int en_off_3d_off=0;
    /* 得出校准后的数据 */
	 if(mpu6050_fc.Cali_3d){
			  mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x - mpu6050_fc.Acc_Offset.x*en_off_3d_off;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y - mpu6050_fc.Acc_Offset.y*en_off_3d_off;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z - mpu6050_fc.Acc_Offset.z*en_off_3d_off;
	 }
   else{	 
    if(sensor_setup.Offset.mpu_flag == 0)
    {
        mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z) ;
    }
    else
    {
        mpu6050_tmp[A_X] = 2*(mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
        mpu6050_tmp[A_Y] = 2*(mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
        mpu6050_tmp[A_Z] = 2*(mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z - 2048) ;
    }
  }
    mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050_fc.Gyro_Offset.x ;//
    mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050_fc.Gyro_Offset.y ;//
    mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050_fc.Gyro_Offset.z ;//


    /* 更新滤波滑动窗口数组 */
    FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];

    for(i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }


    mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


    mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;


    /*坐标转换*/
    Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&mpu6050_fc.Acc.x,&mpu6050_fc.Acc.y,&mpu6050_fc.Acc.z);
    Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&mpu6050_fc.Gyro.x,&mpu6050_fc.Gyro.y,&mpu6050_fc.Gyro.z);

    mpu6050_fc.Gyro_deg.x = mpu6050_fc.Gyro.x *TO_ANGLE;
    mpu6050_fc.Gyro_deg.y = mpu6050_fc.Gyro.y *TO_ANGLE;
    mpu6050_fc.Gyro_deg.z = mpu6050_fc.Gyro.z *TO_ANGLE;

}
