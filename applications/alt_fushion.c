#include "alt_fushion.h"
#include "mymath.h"
#include "filter.h"
#include "imu.h"
#include "quar.h"
#include "mpu6050.h"
#include "usart.h"
#include "height_ctrl.h"
#include "include.h"
#include "rc.h"
#include "ultrasonic.h"
#include "baro_ekf_oldx.h"
#include "eso.h"


void body_to_NEZ(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    body_to_NEZ(vr, v, qc);
}

static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}





u8 OLDX_KF2(float *measure,float tau,float *r_sensor,u8 *flag_sensor,double *state,double *state_correct,float T)
{
float PosDealt;	
float SpeedDealt;
float K_ACC_Z;
float K_VEL_Z;
float K_POS_Z;

if(!flag_sensor[0]&&!flag_sensor[1]&&!flag_sensor[2])	
	return 0;
K_ACC_Z =(5.0f / (tau * tau * tau));
K_VEL_Z =(3.0f / (tau * tau));
K_POS_Z =(3.0f / tau);
//d spd	
if(flag_sensor[0]&&!flag_sensor[1])	
PosDealt=(measure[0]-state[0]);
else if(flag_sensor[0]&&!flag_sensor[1])
PosDealt=measure[1];
else if(flag_sensor[1]&&!flag_sensor[1])
PosDealt=(measure[0]-state[0])+state[1];
else 
return 0;	

state_correct[3*0+2] += r_sensor[0]*PosDealt* K_ACC_Z ;
state_correct[3*0+1] += r_sensor[1]*PosDealt* K_VEL_Z ;
state_correct[3*0+0] += r_sensor[2]*PosDealt* K_POS_Z ;

//acc correct
if(!flag_sensor[1]&&flag_sensor[2])	
state[2]=measure[2]+state_correct[0*3+2];
else if(flag_sensor[1]&&flag_sensor[2])	
state[2]=measure[1]+(measure[2]+state_correct[0*3+2]);
	
//d acc
SpeedDealt=state[2]*T;

//pos correct
state_correct[1*3+0]+=(state[1]+0.5*SpeedDealt)*T;
state[0]=state_correct[1*3+0]+state_correct[0*3+0];

//vel correct
state_correct[0*3+1]+=SpeedDealt;
state[1]=state_correct[1*3+1]+state_correct[0*3+1];

return 1;	
}

float ALT_POS_BMP,ALT_VEL_BMP;
float ALT_POS_SONAR,ALT_VEL_SONAR,ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_BMP_EKF,ALT_VEL_BMP_EKF;
float ALT_POS_BMP_UKF_OLDX,ALT_VEL_BMP_UKF_OLDX,ALT_ACC_BMP_UKF_OLDX;

double P_baro[9]={1,0,0,0,1,0,0,0,1}; 
double X_ukf_baro[3];

double P_barob[16]={1,0,0,0,1,0,0,0,1}; 
double X_ukf_barob[4];
//------------------KF  parameter------------------
float gh_bmp=0.1;
float k_fp_spd_bmp=10;
float gh_bmp1=0.01;
float k_fp_spd_bmp1=10;

float gh_sonar=0.005;
float k_fp_spd_sonar=10;
float ga=0.1;
float gwa=0.1;

float  r_baro_new[4]={0.015,0.05,0.03,5};
float  r_sonar_new[4]={0.036,0.056,0.026,3.5};


double P_kf_baro[9]={1,0,0,1,0,0,1,0,0}; 
double X_kf_baro[3];
double state_correct_baro[6];
double P_kf_baro_bmp[9]={1,0,0,0,1,0,0,0,1}; 
double X_kf_baro_bmp[3];
//float r_baro_ukf[3]={1,1,1};float q_baro_ukf[3]={0.01,0.01,0.01};
float r_baro_ukf[4]={10,1,0.1,0.1};float q_baro_ukf[4]={0.001,0.001,0.001,0.001};

float dead_accz=0.00;
float acc_off_baro=0;
float acc_scale_bmp=1;//0.86;
float k_flt_accz=0.75;
float acc_bmp;

float X_apo_height[2] = {0.0f, 0.0f};
float P_apo_k_height[4] = {100.0f,0.0f,0.0f,100.0f};
float k_bais=  0.0;
float k_bais2= 0;
float r_baro = 10;//10; // 10.0f;			
float r_acc =  0.1; // 0.5f;
float x_tst[2];
float p_tst[2]={1,1};
float kf_tst[2]={1,1};

float k_acc_bais=0;
float acc_body[3];
float acc_bias[3];
float w_z_baro=0.5;
float w_acc_bias=0.05;
float accel_bias_corr[3];

int flag_ero=1;
float Alt_Offset_m1;
int en_bias_fix=0;
float flt_body_acc=0.5;
float k_body_acc=0.3;
float K_SONAR=6;
float acc_est,acc_est_imu;
ESO eso_h_acc,eso_h_spd;
u8 test_bmp=1;
int flag_acc_bias;
float acc_z_att_corr;
float speed_estimation,  bias_accel,  position_estimation;
float  sigmaAcc=0.1,  sigmaPos=1,  gammaAcc=0.1,  gammaBiasAcc=0.1;
void ukf_baro_task1(float T)// 气压计加速度计融合
{
static u8 init,mode_reg;
if(!init)
{   init=1;
		eso_h_spd.h0=eso_h_acc.h0=0.02;
    eso_h_spd.r0=eso_h_acc.r0=4;
}	
float dt=T;
#if SONAR_USE_FC||SONAR_USE_FC1
float  tilted_fix_sonar;
	tilted_fix_sonar=LIMIT((ultra.relative_height/cos(LIMIT(my_deathzoom_21(Pit_fc,5),-45,45)/57.3)/
									cos(LIMIT(my_deathzoom_21(Rol_fc,5),-45,45)/57.3)-ultra.relative_height),0,0.5);
float posz_sonar=LIMIT(ultra.relative_height+tilted_fix_sonar,0,5);
		if( fabs(posz_sonar - ALT_POS_SONAR2) < 0.1 )
		{			
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*4.0f *3.14f *dt ) ) ) *(posz_sonar - ALT_POS_SONAR2) ;
		}
		else if( fabs(posz_sonar - ALT_POS_SONAR2) < 0.2 )
		{
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*2.2f *3.14f *dt ) ) ) *(posz_sonar- ALT_POS_SONAR2) ;
		}
		else if( fabs(posz_sonar - ALT_POS_SONAR2) < 0.4 )
		{
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*1.2f *3.14f *dt ) ) ) *(posz_sonar- ALT_POS_SONAR2) ;
		}
		else
		{
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*0.6f *3.14f *dt ) ) ) *(posz_sonar- ALT_POS_SONAR2) ;
		}	
#endif

float posz;
u8 input_flag=2;
if(mode.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1)){
posz=(float)(baro.relative_height)/1000.;input_flag=1;
}
else
posz=LIMIT(ALT_POS_SONAR2,0,5);

static float temp_r;
u8 i,j;
float acc_temp1,temp;  
float accIn[3];
float acc_body_temp[3];
 		accIn[0] =(float) mpu6050_fc.Acc.x/4096.*9.8-acc_bias[0]*en_bias_fix;//16438.;
		accIn[1] =(float) mpu6050_fc.Acc.y/4096.*9.8-acc_bias[1]*en_bias_fix;//16438.;
		accIn[2] =(float) mpu6050_fc.Acc.z/4096.*9.8-acc_bias[2]*en_bias_fix;//16438.;
    body_to_NEZ(acc_body_temp, accIn, ref_q_imd_down_fc);
    //acc_temp1=(float)(reference_vr_imd_down_fc[2] *mpu6050_fc.Acc.z + reference_vr_imd_down_fc[0] *mpu6050_fc.Acc.x + reference_vr_imd_down_fc[1] *mpu6050_fc.Acc.y - 4096  )/4096.0f*9.8;
		//acc_temp1=acc_body_temp[2]-9.8;

    acc_z_att_corr=-my_sqrt(pow(my_sqrt(pow(sin(Pit_fc*0.0173),2)+pow(sin(Rol_fc*0.0173),2)),2));

		static float wz_acc ;
		static u16 ekf_init_cnt;
		if(ekf_init_cnt++>256 && fabs(acc_temp1)<1)
		{sys_init.baro_ekf=1;}
		
		if(sys_init.baro_ekf)
		{
		wz_acc=firstOrderFilter(acc_body_temp[2]-9.78+acc_z_att_corr*0 ,&firstOrderFilters[ACC_LOWPASS_Z],T);
		//wz_acc+= ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) )*my_deathzoom1( (acc_temp1 - wz_acc),0);
		
//	acc_off_baro =acc_temp1 ;
//	acc_off_baro=LIMIT(acc_off_baro,-3,3);
		}			 
		
		
		
    float corr_baro = flag_ero*( posz- ALT_POS_BMP_UKF_OLDX);
		accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;
    float R_control_now1[9];
		R_control_now1[0]=R_control_now[0][0];R_control_now1[3]=R_control_now[0][1];R_control_now1[6]=R_control_now[0][2];
		R_control_now1[1]=R_control_now[1][0];R_control_now1[4]=R_control_now[1][1];R_control_now1[7]=R_control_now[1][2];
		R_control_now1[2]=R_control_now[2][0];R_control_now1[5]=R_control_now[2][1];R_control_now1[8]=R_control_now[2][2];
		/* transform error vector from NED frame to body frame */
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += PX4_R(R_control_now1, j, i)*accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * w_acc_bias * T;
			}
		}
    
		if(NS==0)
	  acc_off_baro=0;
		else if((Rc_Get_PWM.THROTTLE<1200&&NS==2))		
		{
		acc_off_baro += ( 1 / ( 1 + 1 / ( 3*0.6f *3.14f *dt ) ) ) *(wz_acc- acc_off_baro) ;
		acc_off_baro=LIMIT(acc_off_baro,-3,3);
		}
		
		acc_bias[2]=0;
    static float acc_bais;
		float b[3] = {0.8122  ,  1.6244  ,  0.8122};
		float a[3] = {1.0000  ,  1.5888  ,  0.6600};
		static float xBuf1[3];
		static float yBuf1[3];
	  acc_bmp=(LIMIT(my_deathzoom1(wz_acc-acc_off_baro*1,dead_accz)*acc_scale_bmp,-6.6,6.6));//,xBuf1,yBuf1,a,b,2);//+LIMIT(acc_bais,-1.5,1.5);
    acc_body[2]=acc_bmp-LIMIT(ALT_ACC_BMP_UKF_OLDX,-1,1)*0;
		static float xBuf2[3];
		static float yBuf2[3];
		#define BARO_AV_NUM_FU 100*1/2
		static float baro_av_arr_fu[BARO_AV_NUM_FU];
		static  u16 baro_av_cnt_fu;
		baro.h_origin=((float)(baro.relative_height)/1000.);
		Moving_Average1( (float)( baro.h_origin),baro_av_arr_fu,BARO_AV_NUM_FU, &baro_av_cnt_fu ,&baro.h_flt ); //单位mm/s

    eso_h_spd.h0=eso_h_acc.h0=T;
    OLDX_SMOOTH_IN_ESO(&eso_h_spd,baro.h_flt);
		baro.v_flt=eso_h_spd.v2;
		OLDX_SMOOTH_IN_ESO(&eso_h_acc,baro.v_flt);
		baro.acc_flt=eso_h_acc.v2;
		
		// rotate acc to world frame
   	#if !DEBUG_WITHOUT_SB
		#if USE_RECIVER_MINE==1
		if((Rc_Get.THROTTLE<1200&&NS==2)||ero.baro_ekf)		
		#else
		if((Rc_Get_PWM.THROTTLE<1111&&NS==2)||ero.baro_ekf)		
		#endif
		{
		
		X_ukf_baro[0] =posz;X_ukf_baro[1]=X_ukf_baro[2]=0;acc_bais=0;
		X_kf_baro[0] =posz;X_kf_baro[1]=X_kf_baro[2]=0;
		X_apo_height[0] =posz;X_apo_height[1]=0;

		ero.baro_ekf=0;
		}
			
		if(mode.test3!=mode_reg&&0)
		{
		  if(mode.test3)
			{
			 X_apo_height[0] =ALT_POS_SONAR2;X_apo_height[1]=0;
			}
		  else
			{
			 X_apo_height[0] =(float)(baro.relative_height)/1000.;;X_apo_height[1]=0;
			}	
		}
		mode_reg=mode.test3;
   	#endif 
		
	float gh_use,k_fp_spd_use;
	if(mode.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))
  {gh_use=gh_bmp;k_fp_spd_use=k_fp_spd_bmp;}
  else 
  {gh_use=gh_sonar;k_fp_spd_use=k_fp_spd_sonar;}	
	
	#define BARO_KF2 
	//#define BARO_KF	

	#if defined(BARO_KF2) //KF with limit bias
	u8 flag_sensor[3]={1,0,1};	
	float Z_kf[3]={posz+LIMIT(my_deathzoom1(X_kf_baro[1],0.68),-1,1)*T*k_fp_spd_use,0,acc_bmp};
	if(mode.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))
  OLDX_KF2(Z_kf,r_baro_new[3],r_baro_new,flag_sensor,X_kf_baro,state_correct_baro,T);
	else
	OLDX_KF2(Z_kf,r_sonar_new[3],r_sonar_new,flag_sensor,X_kf_baro,state_correct_baro,T);

	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0];
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1];
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];	
	#elif  defined(BARO_KF) //KF with bias
	double Z_kf[3]={posz+LIMIT(ALT_VEL_BMP_UKF_OLDX,-1,1)*T*k_fp_spd_use*0,0,0};
	kf_oldx( X_kf_baro,  P_kf_baro,  Z_kf,  acc_bmp, gh_use,  ga,  gwa,T);//for spd
	posz=firstOrderFilter(baro.relative_height*0.001 ,&firstOrderFilters[BARO_LOWPASS],T);
	double Z_kf_bmp[3]={posz+LIMIT(my_deathzoom21(ALT_VEL_BMP_UKF_OLDX,0.68),-1,1)*T*k_fp_spd_bmp1*1,0,0};
	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0];	
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1];
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];
	#else  //EKF  without bias
	float Z_baro_ekf[2]={posz+LIMIT(ALT_VEL_BMP_UKF_OLDX,-1,1)*T*k_fp_spd_bmp1*0,acc_bmp};		
	BARO_EKF_OLDX(X_apo_height,P_apo_k_height, X_apo_height, P_apo_k_height ,Z_baro_ekf,  r_baro,  r_acc, T);
	ALT_POS_BMP_UKF_OLDX=X_apo_height[0];
	ALT_VEL_BMP_UKF_OLDX=X_apo_height[1];
	#endif
	
	#if USE_RECIVER_MINE
	if(Rc_Get.THROTTLE<1200&&NS==2)	
	#else
	if(Rc_Get_PWM.THROTTLE<1200&&NS==2)	
	#endif
	#if !DEBUG_WITHOUT_SB
	Alt_Offset_m1=ALT_POS_BMP_UKF_OLDX;
	#endif
	x_tst[0]=kf_tst[0]*x_tst[0]+(1-kf_tst[0])*ALT_VEL_BMP_UKF_OLDX;
	
	static float spd_r,spd_r_imu;
	
	acc_est=(ALT_VEL_BMP_UKF_OLDX-spd_r)/T;
	spd_r=ALT_VEL_BMP_UKF_OLDX;
	if(fabs(acc_est)>10||fabs(ALT_VEL_BMP_UKF_OLDX)>4)	
  {ero.baro_ekf=1;ero.baro_ekf_cnt++;}
	if((fabs(ALT_POS_SONAR2-ALT_POS_BMP_UKF_OLDX)>0.36||(fabs(acc_est)>10||fabs(ALT_VEL_BMP_UKF_OLDX)>4))&&input_flag==2)	
  {ero.baro_ekf=1;ero.baro_ekf_cnt++;}

}