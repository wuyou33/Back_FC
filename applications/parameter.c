
#include "include.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "ctrl.h"
#include "string.h"
#include "ff.h"
#include "height_ctrl.h"

#define SENSOR_SETUP_FILE      "sensor.bin"
#define PID_SETUP_FILE         "pid.bin"
u8 flash_init_error;
sensor_setup_t sensor_setup;
pid_setup_t pid_setup;


static int32_t Para_ReadSettingFromFile(void)
{
	

}

static int32_t Para_WriteSettingToFile(void)
{
	

}


static void  Param_SetSettingToFC(void) 
{
	memcpy(&mpu6050.Acc_Offset,&sensor_setup.Offset.Accel,sizeof(xyz_f_t));
	memcpy(&mpu6050.Gyro_Offset,&sensor_setup.Offset.Gyro,sizeof(xyz_f_t));
	memcpy(&ak8975.Mag_Offset,&sensor_setup.Offset.Mag,sizeof(xyz_f_t));
	memcpy(&mpu6050.vec_3d_cali,&sensor_setup.Offset.vec_3d_cali,sizeof(xyz_f_t));
	
	mpu6050.Acc_Temprea_Offset = sensor_setup.Offset.Acc_Temperature;
	mpu6050.Gyro_Temprea_Offset = sensor_setup.Offset.Gyro_Temperature;
  
  memcpy(&ctrl_1.PID[PIDROLL],&pid_setup.groups.ctrl1.roll,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDPITCH],&pid_setup.groups.ctrl1.pitch,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDYAW],&pid_setup.groups.ctrl1.yaw,sizeof(pid_t));
	
	
	memcpy(&ctrl_2.PID[PIDROLL],&pid_setup.groups.ctrl2.roll,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDPITCH],&pid_setup.groups.ctrl2.pitch,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDYAW],&pid_setup.groups.ctrl2.yaw,sizeof(pid_t));

}

void Para_ResetToFactorySetup(void)
{
// 	/* 加速计默认校准值 */
// 	sensor_setup.Offset.Accel.x = 0;
// 	sensor_setup.Offset.Accel.y = 0;
// 	sensor_setup.Offset.Accel.z = 0;
// 	/* 陀螺仪默认校准值 */
// 	sensor_setup.Offset.Gyro.x = 0;
// 	sensor_setup.Offset.Gyro.y = 0;
// 	sensor_setup.Offset.Gyro.z = 0;
// 	/* 罗盘默认校准值 */
// 	sensor_setup.Offset.Mag.x = 0;		
// 	sensor_setup.Offset.Mag.y = 0;		
// 	sensor_setup.Offset.Mag.z = 0;	
// 	/* 气压计默认校准值 */	
// 	sensor_setup.Offset.Baro = 0;
//    /* 温度默认校准值 */	
// 	sensor_setup.Offset.Acc_Temperature = 0;
// 	sensor_setup.Offset.Gyro_Temperature = 0;
 float pid_att_out[3]={0.7,0.0,0.0};
 float pid_att_in[3]={0.7,0.2,2.2};
 
 float pid_att_out_yaw[3]={0.8,0.05,0.3};
 float pid_att_in_yaw[3]={1.2,0.1,1.2};
 //adrc
 eso_att_inner_c[PITr].eso_dead=eso_att_inner_c[ROLr].eso_dead=1;
 eso_att_inner_c[PITr].b0=eso_att_inner_c[ROLr].b0=20;
 eso_att_inner_c[PITr].not_use_px4=eso_att_inner_c[ROLr].not_use_px4=1;
  /* PID 默认值 */
	pid_setup.groups.ctrl1.pitch.kp = pid_att_in[0];//0.6;
	pid_setup.groups.ctrl1.roll.kp  = pid_setup.groups.ctrl1.pitch.kp;	
	pid_setup.groups.ctrl1.yaw.kp   = pid_att_in_yaw[0];	
	
	pid_setup.groups.ctrl1.pitch.ki = pid_att_in[1];
	pid_setup.groups.ctrl1.roll.ki  = pid_setup.groups.ctrl1.pitch.ki;	
	pid_setup.groups.ctrl1.yaw.ki   = pid_att_in_yaw[1];	
	
	
	pid_setup.groups.ctrl1.pitch.kd = pid_att_in[2];
	pid_setup.groups.ctrl1.roll.kd  = pid_setup.groups.ctrl1.pitch.kd ;	
	pid_setup.groups.ctrl1.yaw.kd   = pid_att_in_yaw[2];	
	
	pid_setup.groups.ctrl1.pitch.kdamp = 1*0.75;//
	pid_setup.groups.ctrl1.roll.kdamp  = pid_setup.groups.ctrl1.pitch.kdamp ;	
	pid_setup.groups.ctrl1.yaw.kdamp   = 1;

  pid_setup.groups.ctrl2.pitch.kp = pid_att_out[0];
  pid_setup.groups.ctrl2.roll.kp  = pid_setup.groups.ctrl2.pitch.kp;	
	pid_setup.groups.ctrl2.yaw.kp   = pid_att_out_yaw[0];	
	
	pid_setup.groups.ctrl2.pitch.ki = pid_att_out[1];
	pid_setup.groups.ctrl2.roll.ki  = pid_setup.groups.ctrl2.pitch.ki;	
	pid_setup.groups.ctrl2.yaw.ki   = pid_att_out_yaw[1];
	
	pid_setup.groups.ctrl2.pitch.kd = pid_att_out[2];
	pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd ;
  pid_setup.groups.ctrl2.yaw.kd   = pid_att_out_yaw[2];
	
	pid_setup.groups.ctrl3.kp = 1.0f;
	pid_setup.groups.ctrl3.ki = 1.0f;
	pid_setup.groups.ctrl3.kd = 1.0f;
	
	pid_setup.groups.ctrl4.kp = 1.0f;
	pid_setup.groups.ctrl4.ki = 1.0f;
	pid_setup.groups.ctrl4.kd = 1.0;
	
	pid_setup.groups.hc_sp.kp = 2.0f;//
	pid_setup.groups.hc_sp.ki = 1.0f;
	pid_setup.groups.hc_sp.kd = 1.0f;
	
	pid_setup.groups.hc_height.kp = 1.0f;
	pid_setup.groups.hc_height.ki = 1.0f;
	pid_setup.groups.hc_height.kd = 1.0f;	
	
	Param_SetSettingToFC();
	PID_Para_Init();
}

void PID_Para_Init()
{
	Ctrl_Para_Init();
	//h_pid_init();

}

void Para_Init()
{
	
	Para_ResetToFactorySetup();
	flash_init_error = 1;

	Param_SetSettingToFC();
	
	PID_Para_Init();
}

void Param_SaveAccelOffset(xyz_f_t *offset)
{
 memcpy(&mpu6050.Acc_Offset,offset,sizeof(xyz_f_t));
 memcpy(&sensor_setup.Offset.Accel, offset,sizeof(xyz_f_t));
	
 sensor_setup.Offset.Acc_Temperature = mpu6050.Acc_Temprea_Offset ;
	
 Para_WriteSettingToFile();
}

void Param_SaveGyroOffset(xyz_f_t *offset)
{
 memcpy(&mpu6050.Gyro_Offset,offset,sizeof(xyz_f_t));
 memcpy(&sensor_setup.Offset.Gyro, offset,sizeof(xyz_f_t));
	
 sensor_setup.Offset.Gyro_Temperature = mpu6050.Gyro_Temprea_Offset ;
	
 Para_WriteSettingToFile();
}

void Param_SaveMagOffset(xyz_f_t *offset)
{
 memcpy(&ak8975.Mag_Offset,offset,sizeof(xyz_f_t));
 memcpy(&sensor_setup.Offset.Mag, offset,sizeof(xyz_f_t));
 Para_WriteSettingToFile();
}

void Param_Save_3d_offset(xyz_f_t *offset)
{
 memcpy(&mpu6050.vec_3d_cali,offset,sizeof(xyz_f_t));
 memcpy(&sensor_setup.Offset.vec_3d_cali, offset,sizeof(xyz_f_t));
	
 Para_WriteSettingToFile();
}

void Param_SavePID(void)
{
 memcpy(&pid_setup.groups.ctrl1.roll,&ctrl_1.PID[PIDROLL],sizeof(pid_t));
 memcpy(&pid_setup.groups.ctrl1.pitch,&ctrl_1.PID[PIDPITCH],sizeof(pid_t));
 memcpy(&pid_setup.groups.ctrl1.yaw,&ctrl_1.PID[PIDYAW],sizeof(pid_t));
  
 memcpy(&pid_setup.groups.ctrl2.roll,&ctrl_2.PID[PIDROLL],sizeof(pid_t));
 memcpy(&pid_setup.groups.ctrl2.pitch,&ctrl_2.PID[PIDPITCH],sizeof(pid_t));
 memcpy(&pid_setup.groups.ctrl2.yaw,&ctrl_2.PID[PIDYAW],sizeof(pid_t));
 Para_WriteSettingToFile();
}
extern u16 flash_save_en_cnt;

void Parameter_Save()
{
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
