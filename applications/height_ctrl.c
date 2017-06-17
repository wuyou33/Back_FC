#include "height_ctrl.h"
#include "mymath.h"
#include "filter.h"
#include "ctrl.h"
#include "alt_fushion.h"
#include "ultrasonic.h"
u8 height_ctrl_mode=0;
int ultra_distance;
int exp_height;
float	set_height_e,set_height_em,
			set_speed_t,set_speed,exp_speed,fb_speed,
			exp_acc,fb_acc,fb_speed,fb_speed_old;

_hc_value_st hc_value;


#define EXP_Z_SPEED  ( 5.26f *my_deathzoom1( (thr-500), 75 ) )//----------ң�ض�Ӧ�����ٶ�
float rng_time1;
float HOLD_THR =450+100; //��ͣ����
float ALT_HOLD_THR_RANGE_SCALE=2;
float exp_spd_zv,thr_use,thr_in_view;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid_v wz_speed_pid_v_safe;
_st_height_pid_v wz_speed_pid_v_eso;

_st_height_pid_v ultra_ctrl,ultra_ctrl_safe;

_st_height_pid wz_speed_pid,wz_speed_pid_safe;
_st_height_pid wz_speed_pid_use;
_st_height_pid ultra_pid_safe,ultra_pid;
 _st_height_pid ultra_pid_use;

#define MAX_HEIGH_ERO 1000
float exp_height_speed;
float exp_height_speed_safe,exp_height_safe;
float ultra_speed,ultra_speed_safe;
float ultra_dis_lpf,ultra_dis_lpf_safe;
float ultra_ctrl_out_safe, ultra_ctrl_out,ultra_ctrl_out_use;
float baro_speed_ano;
float height_ctrl_out;
float wz_acc;
float adrc_out;
void Ultra_PID_Init()
{
	HOLD_THR =HOLD_THR_PWM;//��--------------------�޸���ͣ����  ���ݲ�ͬ����������������ʵ�� 
//-------------------�⻷PID������ʼ��
	ultra_pid.kp = 1.0;//0.88;
	ultra_pid.ki = 0.1;
	ultra_pid.kd = 0.0;
//------------------��ȫģʽ ֻ���ٶȻ�
  ultra_pid_safe.kp = 0;
	ultra_pid_safe.ki = 0.0;
	ultra_pid_safe.kd = 0.0;
	//adrc
	eso_pos[Zr].b0=0;
	eso_pos[Zr].eso_dead=5;
	eso_pos[Zr].eso_for_z=1;
}

  
void WZ_Speed_PID_Init()
{//use
	  Ultra_PID_Init();
	  wz_speed_pid.kp = 0.5;
		wz_speed_pid.ki = 0.3;
		wz_speed_pid.kd = 1.68;
	  wz_speed_pid.fp=0.2;
	//------------------------------��ȫģʽPID��ʼ��
		wz_speed_pid_safe.kp = 0.5;
		wz_speed_pid_safe.ki = 0.2;
		wz_speed_pid_safe.kd = 1.68; 
	  wz_speed_pid_use.fp=0.2;
	//adrc
	  eso_pos_spd[Zr].b0=15;
	  eso_pos_spd[Zr].eso_dead=0;	
	  eso_pos_spd[Zr].eso_for_z=1;
}

#define BARO_SPEED_NUM 50
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt[2];
u8 switch_r;
int wz_acc_ukf;
void height_mode_switch(void)//��ѹ�� �������Զ��л�
{
static u8 state;
static u16 cnt,cnt1;	
}


float sonar_spd;
u8 BARO_HIHG_NUM=1;//10;//30;
float baro_h_arr[100 + 1];
u16 baro_h_cnt[2];
float baro_h_arr1[100 + 1];
u16 baro_h_cnt1[2];
float k_temp=12; 
u8 hs_ctrl_cnt_max[2]={3,1},hold_alt_flag;
float out_timer_high,in_timer_high;
float baro_only_move,baro_only_move_flt;
float ano_temp;
float k_acc_ukf=10;//20;
u8 mid_usp=5;
u8 EN_TUO_ACC_FIX_H=0;
float off_inner=2;
float wz_acc_ukf1,wz_acc1;
float ultra_speed_bmp;
float tilted_fix1;
float k_flt_z_spd=3.25;
void Height_Ctrl1(float T,float thr)
{			static u8 hs_ctrl_cnt;
	static float wz_speed_t;
	static u8 height_ctrl_start_f,height_mode_reg;
	static u16 hc_start_delay;
	float t_sonar_out;
	float temp;
	float t_h=(float)Get_Cycle_T(GET_T_HIGH_CONTROL)/1000000.+0.0001;
	static u8 init ;
	if(!init){init=1;WZ_Speed_PID_Init();}
	
	switch( height_ctrl_start_f )
	{		
		case 0:
    //thr_hold_estimate( T, thr,fly_ready, fly_ready);
		#if EN_ATT_CAL_FC //���ʹ��FCģ�������̬
		temp=(reference_vr_imd_down_fc[2] *mpu6050_fc.Acc.z + reference_vr_imd_down_fc[0] *mpu6050_fc.Acc.x + reference_vr_imd_down_fc[1 ] *mpu6050_fc.Acc.y - 4096  );
		#else
		temp=(reference_vr_imd_down[2] *mpu6050.Acc.z + reference_vr_imd_down[0] *mpu6050.Acc.x + reference_vr_imd_down[1 ] *mpu6050.Acc.y - 4096  );
		#endif
		
		//��ͨ�˲����㴹ֱ������ٶ�
		wz_acc_ukf1 += ( 1 / ( 1 + 1 / ( k_acc_ukf *3.14f *T ) ) ) *my_deathzoom1( (temp - wz_acc_ukf1),0 );
		wz_acc1 += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom1( (temp - wz_acc1),0);
		Moving_Average1( (float)( baroAlt),baro_h_arr,BARO_HIHG_NUM, baro_h_cnt ,&baro_only_move ); //��λmm/s
		Moving_Average1( (float)( baroAlt),baro_h_arr1,85, baro_h_cnt1 ,&baro_only_move_flt ); //��λmm/s
	if(mode.en_h_mode_switch)
				height_mode_switch();
	
	    if( hs_ctrl_cnt++>=4)//PA
			{  //----------------------mode switch----------------------
				in_timer_high=(float)Get_Cycle_T(GET_T_HIGH_CONTROL_I)/1000000.;hs_ctrl_cnt=0;
				if(height_mode_reg==1&&height_ctrl_mode==2)//SONAR<-BMP
				{exp_height=ultra_dis_lpf=ALT_POS_SONAR2*1000;}
				else if(height_mode_reg==2&&height_ctrl_mode==1)//SONAR->BMP
				{exp_height=ultra_dis_lpf=baro.h_flt;}
				 height_mode_reg=height_ctrl_mode;
				//---------------------safe height--------------------------------
				static u8 safe_mode_reg;
				 if(safe_mode_reg==0&&mode.height_safe==1)//SONAR->BMP
				{exp_height_safe=ultra_dis_lpf_safe=baro.h_flt;}
				else if(safe_mode_reg==1&&mode.height_safe==0)
				{
				if(height_ctrl_mode==2)//SONAR<-BMP
				{exp_height=ultra_dis_lpf=ALT_POS_SONAR2*1000;}
				else if(height_ctrl_mode==1)//SONAR->BMP
				{exp_height=ultra_dis_lpf=baro.h_flt;}
				
				}safe_mode_reg=mode.height_safe;
				static u8 state_thr,cnt_down;
				static float thr_reg;
				thr_in_view=thr;
				
				thr_use=thr;//����ʹ��ң����
				if(mode.height_safe)//��ȫģʽ��PID����ѡ��
				{
				mode.height_in_speed=1;
				ultra_pid_use.kp =ultra_pid_safe.kp;ultra_pid_use.ki =ultra_pid_safe.ki; ultra_pid_use.kd =ultra_pid_safe.kd;  
				wz_speed_pid_use.kp =wz_speed_pid_safe.kp;wz_speed_pid_use.ki =wz_speed_pid_safe.ki; wz_speed_pid_use.kd =wz_speed_pid_safe.kd;  
				wz_speed_pid_use.fp=wz_speed_pid_safe.fp;
				}
				else
				{
				ultra_pid_use.kp =ultra_pid.kp;ultra_pid_use.ki =ultra_pid.ki; ultra_pid_use.kd =ultra_pid.kd;  
				wz_speed_pid_use.kp =wz_speed_pid.kp;wz_speed_pid_use.ki =wz_speed_pid.ki; wz_speed_pid_use.kd =wz_speed_pid.kd; 
				wz_speed_pid_use.fp=wz_speed_pid.fp;	
				}
				 //------------------------SPD CONTOLLER------------------
				     exp_spd_zv=EXP_Z_SPEED;//������
					
				
						if( ABS(hc_value.fusion_speed) < 100 )
						{
						ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *in_timer_high*k_flt_z_spd ) ) ) * ( (float)(hc_value.fusion_speed) - ultra_speed );
						}
						else
						{
						ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *in_timer_high*k_flt_z_spd  ) ) ) * ( (float)(hc_value.fusion_speed) - ultra_speed );
						}
						 //ultra_speed=LIMIT(ultra_speed,-2.5*1000,2.5*1000);
				     ultra_speed=LIMIT(hc_value.fusion_speed,-2.5*1000,2.5*1000);
				   if(smart.rc.POS_MODE==SMART_MODE_SPD)//only for smart_spd
					 {
						 if(smart.spd.z==0)
						 ultra_ctrl_out_use=ultra_ctrl_out; 
					 } 
					 else{
				     if(!hold_alt_flag||mode.height_safe)//||
							// (height_ctrl_mode==1&&(fabs(CH_filter[0])>25||fabs(CH_filter[1])>25)))
						 ultra_ctrl_out_use=EXP_Z_SPEED;
						 else
						 ultra_ctrl_out_use=ultra_ctrl_out; 
           }
						 
					 if((ALT_POS_SONAR2<SONAR_HEIGHT&&(NAV_BOARD_CONNECT||ultra.measure_ok))&&ultra_ctrl_out_use<0&&!mode.height_safe&&0)//���������������
						 ultra_ctrl_out_use=LIMIT(ultra_ctrl_out_use,-100,1000);
					 if(ALT_POS_SONAR2>4&&height_ctrl_mode==2)
						 ultra_ctrl_out_use=LIMIT(ultra_ctrl_out_use,-1000,0);
						 height_speed_ctrl1(in_timer_high,thr_use,LIMIT(ultra_ctrl_out_use,-1000,1000),ultra_speed);	//�ٶȻ� 
			}//---end of speed control
			static u8 cnt_100ms;
			#if  defined(SONAR_SAMPLE1)
			hs_ctrl_cnt_max[0]=10;//50Hz  WT
			#endif
			if( cnt_100ms++>=8)//PA
			{
	      out_timer_high=(float)Get_Cycle_T(GET_T_HIGH_CONTROL_O)/1000000.;
				cnt_100ms=0;
				Ultra_Ctrl1(out_timer_high,thr_use);//λ�û�
			}
			
			
			if(height_ctrl_mode)//����ģʽ
		{	
			  height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else//�ֶ�ģʽ
		{		
		  	height_ctrl_out = thr-50*LIMIT(wz_acc_ukf1/4096,-0.2,0.2)*9.8;   
		}		
		break; 
		default: break;		
	} //switch
}
#define MIN_THR_DOWN HOLD_THR-50
#define MIN_THR_RC_CHECK 200 //RC MIN_THR_CHECK
int Thr_down_min_portect(int thr_in,float T){//�����½��������� δʹ��
static u8 state;
int out;
//state
	switch(state)	
	{
		case 0:if(!Thr_Low&&thr_in>MIN_THR_DOWN)//Check RC
						state=1;
			break;
		case 1:if(500 + CH_filter[THRr]<MIN_THR_RC_CHECK||!fly_ready)
						state=0;
			break;
		default:state=0; break;
	}
//output
	switch(state)	
	{
		case 0:out=thr_in;break;
		case 1:out=LIMIT(thr_in,MIN_THR_DOWN,HOLD_THR);break;
		default:out=thr_in;break;
	}
return out;
}	
///test 
float k_d=1;
float p1=0.4,p2=0.1;//0.35;//0.3;//WT
u8 speed_ctrl_sel=1,speed_spd_sel=1;
float wz_speed_old,wz_acc_mms2;
float height_thrv,wz_speed_pid_v_view,wz_speed_pid_v_view_eso;
float d_view;
float ero_view[2];
float H_INT_REG;
u8 int_save=0;
void height_speed_ctrl1(float T,float thr,float exp_z_speed,float h_speed)//�ٶȿ���
{
static float thr_lpf;
float height_thr;
float wz_acc_mms21;
static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2,hc_acc_i;

	wz_acc_mms2 = (wz_acc_ukf1/4096.0f) *9800;//-acc_bais*1000;//
  //wz_acc_mms2 = acc_body[2]*1000;
	if(!fly_ready||Thr_Low)wz_speed_pid_v.err_i=0;
	else if(fly_ready&&EXP_Z_SPEED>0&&!int_save){
		H_INT_REG=(float)H_INT/100.;
		int_save=1;
	}
	height_thr = LIMIT( ALT_HOLD_THR_RANGE_SCALE * thr , 0, HOLD_THR );

	thr_lpf += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	height_thrv=thr_lpf;

	wz_speed_pid_v.now=wz_speed=(h_speed);
  wz_speed_pid_v.exp=exp_z_speed;
  //-------------------------------------------PID-------------------------------
	wz_speed_pid_v.err = wz_speed_pid_use.kp *( exp_z_speed - wz_speed );
	wz_speed_pid_v.err_weight = (float)ABS(wz_speed_pid_v.err)/MAX_VERTICAL_SPEED;
	wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid_use.kd * (-my_deathzoom1( (LIMIT(wz_acc_mms2,-9800*0.8,9800*0.8) ) ,25)) *T;
	if(fabs(wz_speed_pid_v.err)<eso_pos_spd[Zr].eso_dead||eso_pos_spd[Zr].b0==0||mode.en_eso_h_in==0||mode.height_safe==1||1){
	wz_speed_pid_v.err_i += wz_speed_pid_use.ki *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i+H_INT*Thr_Weight*0,-Thr_Weight *300,Thr_Weight *300);}
	else
	wz_speed_pid_v.err_i=0;
	
	if(fly_ready&&wz_speed_pid_v.pid_out>300&&fabs(exp_z_speed-wz_speed)<300&&int_save>=1){
	H_INT_REG= ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( wz_speed_pid_v.err_i - H_INT_REG );
	int_save=2;
	}
	if(!fly_ready&&Thr_Low&&int_save==2)
	{
	int_save=0;
	H_INT=LIMIT(H_INT_REG*100*0.3+0.7*H_INT,-88,88);
	// WRITE_PARM();	
	}
	OLDX_POS_CONTROL_ESO(&eso_pos_spd[Zr],exp_z_speed,wz_speed,eso_pos_spd[Zr].u,T,400,wz_speed_pid_use.kp,thr_view);//�ٶȻ��Կ��ſ���
	
	if(mode.en_eso_h_in&&!mode.height_safe&&eso_pos_spd[Zr].b0!=0){//ADRC
		wz_speed_pid_v.pid_out=thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid_use.fp *LIMIT((0.45f + 0.55f*wz_speed_pid_v.err_weight),0,1)*exp_z_speed + 
	(1 - wz_speed_pid_use.fp )*(  wz_speed_pid_v.err_d+wz_speed_pid_v.err_i+eso_pos_spd[Zr].u)  ),-400,400);
	}
  else{	
 	wz_speed_pid_v.pid_out=thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid_use.fp *LIMIT((0.45f + 0.55f*wz_speed_pid_v.err_weight),0,1)*exp_z_speed + 
	(1 - wz_speed_pid_use.fp )*(  wz_speed_pid_v.err_d+wz_speed_pid_v.err + wz_speed_pid_v.err_i)  ),-400,400);	
	  }
	if(!fly_ready||force_Thr_low)
	wz_speed_pid_v.pid_out=0;	
	wz_speed_pid_v.err_old =wz_speed;
}

u8 baro_ctrl_start;
float k_flt_pos_z=1;
float baro_height,ultra_speed_bmp;
float ultra_sp_test[2];
float k_dh=1;
void Ultra_Ctrl1(float T,float thr)//λ�û�PID
{
	float ultra_sp_tmp,ultra_dis_tmp;	
	#define MID_THR 500 //ҡ����λPWM
  static int bmp_expr;
	static float h_reg[2];
	static u16 cnt_in_mid;
	static u8 state_high_set=0;
	
	 #define CHECK_CNT_MID 0.25

  if(EXP_Z_SPEED!=0&&smart.rc.POS_MODE==0)
	{  hold_alt_flag=0;
	if(height_ctrl_mode==1)
		{
		#if EN_ATT_CAL_FC	
		exp_height=ALT_POS_BMP_UKF_OLDX*1000;
		#else
		exp_height=ALT_POS_BMP_UKF_OLDX*1000;	
		#endif
	  }
		if(height_ctrl_mode==2)
		{exp_height=LIMIT(ALT_POS_BMP_UKF_OLDX,0.04,10)*1000;}
	}
	else
		hold_alt_flag=1;
	
//	if(height_ctrl_mode==1&&(fabs(CH_filter[0])>25||fabs(CH_filter[1])>25)&&smart.rc.POS_MODE==0)
//	#if EN_ATT_CAL_FC		
//	exp_height=ALT_POS_BMP_UKF_OLDX*1000;
//	#else
//	exp_height=ALT_POS_BMP_UKF_OLDX*1000;
//	#endif
	
	if(mode_change&&smart.rc.POS_MODE==0){mode_change=0;
	if(height_ctrl_mode==1)
	#if EN_ATT_CAL_FC	
	{exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
	#else
	{exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
	#endif
	if(height_ctrl_mode==2)
	{exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
	}
	static u8 mode_safe_reg;
	if(mode.height_safe&&!mode_safe_reg){mode_change=1;
	#if EN_ATT_CAL_FC		
	exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
	#else
	exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
  #endif
	mode_safe_reg=mode.height_safe;
	
	if(smart.rc.POS_MODE==0){
	if(height_ctrl_mode==2&&!mode.height_safe&&ALT_POS_SONAR2*1000>SONAR_HEIGHT*1.5&&mode.h_is_fix){//������ģʽ�����߶ȹ̶�Ϊ1.2m
	if(circle.connect)
	exp_height=900;
	else
	exp_height=900;	
  }}
	
	if(height_ctrl_mode==1||mode.height_safe)
	#if EN_ATT_CAL_FC	
	ultra_dis_tmp=ALT_POS_BMP_UKF_OLDX*1000;
	#else
	ultra_dis_tmp=ALT_POS_BMP_UKF_OLDX*1000;	
	#endif
	else{ 
	float tilted_fix_sonar;	
	#if EN_ATT_CAL_FC
	tilted_fix_sonar=LIMIT((ALT_POS_BMP_UKF_OLDX/cos(LIMIT(my_deathzoom_21(Pit_fc,5),-45,45)/57.3)/
									cos(LIMIT(my_deathzoom_21(Rol_fc,5),-45,45)/57.3)-ALT_POS_BMP_UKF_OLDX),0,0.5);
	ultra_dis_tmp=  (ALT_POS_BMP_UKF_OLDX+tilted_fix_sonar*0)*1000;
	#else
	tilted_fix_sonar=LIMIT((ALT_POS_SONAR2/cos(LIMIT(my_deathzoom_21(Pitch,5),-45,45)/57.3)/
							cos(LIMIT(my_deathzoom_21(Roll,5),-45,45)/57.3)-ALT_POS_SONAR2),0,0.5);
	ultra_dis_tmp=  (ALT_POS_SONAR2+tilted_fix_sonar*1)*1000;
	#endif		
		
	}	
 
//		if( fabs(ultra_dis_tmp - ultra_dis_lpf) < 0.1*1000 )
//		{			
//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( k_flt_pos_z*4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
//		}
//		else if( fabs(ultra_dis_tmp - ALT_POS_SONAR2) < 0.2*1000 )
//		{
//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( k_flt_pos_z*2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
//		}
//		else if( fabs(ultra_dis_tmp - ALT_POS_SONAR2) < 0.4*1000 )
//		{
//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( k_flt_pos_z*1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
//		}
//		else
//		{
//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( k_flt_pos_z*0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
//		}	
	ultra_ctrl.now=ultra_dis_lpf=  ultra_dis_tmp;	
	if((smart.rc.POS_MODE==SMART_MODE_SPD&&fabs(smart.spd.z)>0)||(smart.rc.POS_MODE==SMART_MODE_RC&&fabs(smart.rc.THROTTLE-1500)>25))
	exp_height=ultra_dis_lpf;
		
	ultra_ctrl.exp=exp_height;
	
		
	if(ultra_pid.ki==0||(mode.use_dji)||!fly_ready)ultra_ctrl.err_i=0;
	if(height_ctrl_mode==1||mode.height_safe)
	ultra_ctrl.err = ( ultra_pid_use.kp*1*LIMIT(my_deathzoom1(exp_height - ultra_dis_lpf,5),-800,800) ); 
	else
	ultra_ctrl.err = ( ultra_pid_use.kp*LIMIT(my_deathzoom1(exp_height - ultra_dis_lpf,5),-800,800) );
	
	ultra_ctrl.err_d = ultra_pid.kd *( 0.0f *(-wz_speed*T) + 1.0f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
	
	if(eso_pos[Zr].b0==0){
	ultra_ctrl.err_i += ultra_pid_use.ki *ultra_ctrl.err *T;
	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
  }
	else
	ultra_ctrl.err_i=0;	
	
	OLDX_POS_CONTROL_ESO(&eso_pos[Zr],(exp_height),(ultra_dis_lpf),eso_pos[Zr].u,T,800,ultra_pid_use.kp,thr_view);//ADRC

	if(mode.en_eso_h_in&&!mode.height_safe&&eso_pos[Zr].b0!=0)
  ultra_ctrl.pid_out = eso_pos[Zr].u  + ultra_ctrl.err_i + ultra_ctrl.err_d;	 
	else	
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;

	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-1000,1000);
	ultra_ctrl_out = ultra_ctrl.pid_out;
	ultra_ctrl.err_old = ultra_ctrl.err;
}

