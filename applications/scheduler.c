
#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "bmp.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "fly_mode.h"
#include "anotc_baro_ctrl.h"
#include "alt_fushion.h"

u16 Rc_Pwm_In[8];
s16 loop_cnt;


loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}

void Duty_1ms()
{
}

float inner_loop_time,inner_loop_time_yaw;
float test[5];	
float Rol_fc1,Pit_fc1,Yaw_fc1;
void Duty_2ms()
{
	static u8 cnt;
  float temp;
	temp = Get_Cycle_T(GET_T_INNER)/1000000.0f; 						//获取内环准确的执行周期
	if(temp<0.001||temp>0.003)
		inner_loop_time=0.002;
	else
		inner_loop_time=temp;
	
	#if EN_ATT_CAL_FC
	MPU6050_Read(); 															//读取mpu6轴传感器

	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理
	if(cnt++>4){cnt=0;
	inner_loop_time_yaw = Get_Cycle_T(GET_T_IMU_YAW)/1000000.0f;	
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
	//if(!NAV_BOARD_CONNECT)
 	IMUupdate(0.5f *inner_loop_time_yaw,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z
	,&Rol_fc,&Pit_fc,&Yaw_fc1);
  }
	if(NAV_BOARD_CONNECT)
		Yaw_fc=Yaw;
	else
		Yaw_fc=Yaw_fc1;
		
//	MadgwickAHRSupdate(inner_loop_time,my_deathzoom_21(mpu6050_fc.Gyro_deg.x,0.5)/57.3, my_deathzoom_21(mpu6050_fc.Gyro_deg.y,0.5)/57.3, 
//	my_deathzoom_21(mpu6050_fc.Gyro_deg.z,0.5)/57.3,(float)mpu6050_fc.Acc.x/4096., (float)mpu6050_fc.Acc.y/4096., (float)mpu6050_fc.Acc.z/4096.,
//	0,0,0,
//	&Rol_fc,&Pit_fc,&Yaw_fc_q);//计算俯仰和横滚
	#endif
	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
}

float outer_loop_time;
void Duty_5ms()
{ static u8 cnt;
	float temp;
	temp = Get_Cycle_T(GET_T_OUTTER)/1000000.0f;								//获取外环准确的执行周期
	if(temp<0.004||temp>0.006)
		outer_loop_time=0.005;
	else
		outer_loop_time=temp;
	
 	CTRL_2( outer_loop_time ); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
}

float pos_time;
float baro_task_time;
u8 UART_UP_LOAD_SEL=2;//<------------------------------上传数据选择
u8 force_flow_ble_debug;
void Duty_10ms()
{
	static u8 cnt_bmp;
	static u8 cnt[4];					 		

				//To  Odroid 图像模块
				if(cnt[0]++>2){cnt[0]=0;
						#if EN_DMA_UART3 
					if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA2_Steam7传输完成标志
							data_per_uart3();
						  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3);     //开始一次DMA传输！	
								}	
						#else
								;//UsartSend_GPS(state_test);//Send_IMU_TO_GPS();	
						#endif
							}			
				
				//To  IMU模块	
				if(cnt[1]++>0){cnt[1]=0;	
				  #if EN_DMA_UART2 					
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
							data_per_uart2();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //开始一次DMA传输！	
								}	
					#else
								 GOL_LINK_TASK();	
					#endif
							}					
							
				//BLE UPLOAD《----------------------蓝牙调试
					if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
							{ 	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
								  SendBuff1_cnt=0;
									#if USE_BLE_FOR_APP			  
									APP_LINK();
									#endif		  
							if(cnt[2]++>1){cnt[2]=0;
								    if(mode.att_pid_tune){//PID TUNING
											{	
										 #if !TUNING_Z									
												if(ctrl_2.PID[PIDROLL].kp!=0&&KEY[7])//OUTTER
												data_per_uart1(
												#if TUNING_X
												0,-except_A.x*10,0,
												#else
												0,except_A.y*10,0,
												#endif
												#if EN_ATT_CAL_FC
													#if TUNING_X
													0,-Rol_fc*10,0,
													#else
													0,Pit_fc*10,0,
													#endif
												#else
												0,-Roll*10,0,
												#endif
												-ctrl_2.err.y*10,0*10,0,
												(int16_t)(0*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
												else//INNER
												data_per_uart1(
												#if TUNING_X
												0,-except_AS.x,0,
												#else
												0,except_AS.y,0,
												#endif
												#if TUNING_X
												0,-mpu6050_fc.Gyro_deg.x,0,
												#else
												0,-mpu6050_fc.Gyro_deg.y,0,
												#endif 
												-ctrl_1.err.y,0,0,
												(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
												}
											#else
											  if(ctrl_2.PID[PIDYAW].kp!=0&&KEY[7])//OUTTER
												data_per_uart1(
												0,-except_A.z*10,0,
												0,-Yaw_fc*10,0,
												-ctrl_2.err.z*10,0*10,0,
												(int16_t)(0*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
												else//INNER
												data_per_uart1(
												0,-except_AS.z,0,
												0,-mpu6050_fc.Gyro_deg.z,0,
												-ctrl_1.err.z,0,0,
												(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
												}				
											#endif
										}
										else if(flow_debug.en_ble_debug||force_flow_ble_debug)//DEBUG  FLOW
											data_per_uart1(
											flow_debug.ax,flow_debug.ay,flow_debug.az,
										  flow_debug.gx,flow_debug.gy,flow_debug.gz,
										  flow_debug.hx,flow_debug.hy,flow_debug.hz,
											(int16_t)(inner_loop_time*10000.0),(int16_t)(outer_loop_time*10000.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
										else{//DEBUG-------------------------Normal mode--------------------------------
								    switch(UART_UP_LOAD_SEL)
											{
											case 0://BMP UKF
											data_per_uart1(
											baroAlt/10,baro.relative_height/10,hc_value.fusion_height/10,
											ALT_VEL_BMP_EKF*100,hc_value.fusion_speed/10,ultra_speed/10,
											ALT_POS_SONAR2*100,0*100,hc_value.fusion_acc*100,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 1://BMP UKF
											data_per_uart1(
											nav_spd_ctrl[Y].pid_out*10,nav_spd_ctrl[X].pid_out*10,ALT_POS_SONAR2*100,
											VEL_UKF_Y*100,VEL_UKF_X*100,ultra_speed/10,
											acc_body[Y]/100.,acc_body[X]/100.,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 2://BMP UKF
											data_per_uart1(
											0,ultra_ctrl.exp,ultra_ctrl.now,
											0, wz_speed_pid_v.exp,wz_speed_pid_v.now,
											VEL_UKF_Y*100,VEL_UKF_X*100,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											default:break;
											}
										}
									}
							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //开始一次DMA传输！	  
							}	
			
				
						
				//To  SD卡
						static u8 sd_sel;
				if(cnt[3]++>0){cnt[3]=0;
				
			     #if EN_DMA_UART4 			
					if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
								{ 
							DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);
							
//							if(!mode.en_sd_save)
//							data_per_uart4(SEND_DEBUG);
//						  else	
							clear_nrf_uart();		
							nrf_uart_cnt=0;
						  sd_publish();			
							switch(sd_sel){
							case 0:sd_sel=1;		
							data_per_uart4(SEND_SD_SAVE1);	
							break;
							case 1:sd_sel=2;
							data_per_uart4(SEND_SD_SAVE2);
							break;
							case 2:sd_sel=0;
							data_per_uart4(SEND_SD_SAVE3);
							break;
//							case 3:sd_sel=4;
//							data_per_uart4(SEND_MARKER);
//							break;
//							case 4:sd_sel=0;
//							data_per_uart4(SEND_IMU);//data_per_uart4(SEND_DEBUG);
							
							}
							USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
							MYDMA_Enable(DMA1_Stream4,nrf_uart_cnt+2);   
								}		
					#else
							SD_LINK_TASK2(SEND_IMU);	
					#endif
							}		
}

void Duty_20ms()
{
    float temp =(float) Get_Cycle_T(GET_T_BARO_UKF)/1000000.;							
		if(temp<0.015||temp>0.025)
		baro_task_time=(float)20/1000.;	
		else
		baro_task_time=temp;
    
	  baro_ctrl(baro_task_time,&hc_value);			
		pos_time=baro_task_time;
	  //pos_time =(float) Get_Cycle_T(GET_T_EKF)/1000000.;	

	  Positon_control(pos_time);
		//------------------------RC UPDATE-----------------------  	
		if(Rc_Get_PWM.update){
		RX_CH_PWM[THRr]=	LIMIT(Rc_Get_PWM.THROTTLE-RX_CH_FIX_PWM[THRr]-3,1000,2000)	;
		RX_CH_PWM[ROLr]=  my_deathzoom_rc(Rc_Get_PWM.ROLL-RX_CH_FIX_PWM[ROLr]-3,5)	;
		RX_CH_PWM[PITr]=  my_deathzoom_rc(Rc_Get_PWM.PITCH-RX_CH_FIX_PWM[PITr]-3,5)	;
		RX_CH_PWM[YAWr]=  my_deathzoom_rc(Rc_Get_PWM.YAW-RX_CH_FIX_PWM[YAWr]-3,5)	;

		RX_CH_PWM[AUX3r]=Rc_Get_PWM.POS_MODE;
		RX_CH_PWM[AUX4r]=Rc_Get_PWM.HEIGHT_MODE;
	  }
		else
		{
	  RX_CH_PWM[THRr]=	1000;
	  RX_CH_PWM[ROLr]=  1500;
	  RX_CH_PWM[PITr]=  1500;
		RX_CH_PWM[YAWr]=  1500;
		}	
}

void Duty_50ms()
{
		 //if(cnt_loss_nrf++>1500/50){cnt_loss_nrf=1500/50+1;loss_nrf=1;}
		if(imu_loss_cnt++>1500/50){imu_loss_cnt=1500/50+1;NAV_BOARD_CONNECT=0;}
		 
		//---------------use now
		//------------0 1   |   2 3       KEY_SEL
		#if USE_RECIVER_MINE		
		mode.flow_hold_position=KEY_SEL[0];
		mode.height_safe=KEY_SEL[1];
    #else
    mode.en_sonar_avoid=KEY_SEL[0];		
    mode.en_sd_save=KEY_SEL[1];		
    #endif
		
		mode.en_pid_sb_set=KEY_SEL[2];//使能PID设置	
//-------------------------------------------------	
		#if !USE_RECIVER_MINE
			#if !USE_TOE_IN_UNLOCK
			if(Rc_Get_PWM.RST>1500&&Rc_Get_PWM.update&&Rc_Get_PWM.THROTTLE>1000)
			fly_ready=1;
			else
			fly_ready=0;
			#endif
		#else
			#if  DEBUG_WITHOUT_SB
			if(cnt2++>200)//
			{fly_ready=1;cnt2=200+1;}
			#else
				#if !USE_RC_GROUND&&!USE_TOE_IN_UNLOCK
					if(Rc_Get_PWM.RST>1500)
						fly_ready=1;
						else
						fly_ready=0;
				#else
				fly_ready=KEY_SEL[3];//解锁
				#endif
			#endif
		#endif
					
	  //------------7 6 5 4  |  3 2 1 0  KEY
		//mode.trig_flow_spd= KEY[7];//1
		//mode.trig_h_spd=KEY[4];
	  mode.imu_use_mid_down=1;//KEY_SEL[1];
    //mode.baro_f_use_ukfm=KEY[7];//KEY[7];			
		//mode.flow_f_use_ukfm=KEY[5];//
		if(mode.flow_hold_position==2)
		mode.h_is_fix=1;		
		else
		mode.h_is_fix=0;			
		mode.en_eso_h_in=1;
		mode.yaw_use_eso=0;//KEY[7];
		mode.test4=1;//pos acc use
		mode.flow_f_use_ukfm=2;
		
		
			#if USE_TOE_IN_UNLOCK
//				if(Rc_Get_PWM.RST>1500 )
//					
//					else
//					mode.test4=0;
				
			//if(Rc_Get_PWM.RST>1500 )
			
//			else
//			mode.flow_f_use_ukfm=0;
			#endif
		 if(mode.flow_hold_position==2&&circle.connect)	
     mode.rc_control_flow_pos_sel=1;
		 else if(mode.flow_hold_position==2)	
     mode.rc_control_flow_pos_sel=2;
		 else
		 mode.rc_control_flow_pos_sel=0; 
		#if !USE_RECIVER_MINE
		//	if(Rc_Get_PWM.AUX1>1500&&ALT_POS_SONAR2<3)
			mode.baro_f_use_ukfm=0;
		#endif	
		
			//}
		mode.att_pid_tune=KEY[6]&&KEY[5]&&KEY[3]&&KEY[2]&&KEY[1]&&KEY[0];

	mode_check(CH_filter,mode_value);
  if(!NAV_BOARD_CONNECT)			
	ANO_AK8975_Read();
	#if SONAR_USE_FC
	if((!Thr_Low)||NS==0)
	Ultra_Duty();
	#endif
	if(circle.lose_cnt++>4/0.05)
	circle.connect=0;
	if(marker.lose_cnt++>4/0.05)
	marker.connect=0;
	circle.use_spd=circle.connect&&mode.flow_sel;
	
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

