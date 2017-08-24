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
#include "oldx_kf2.h"

float acc_z_acc[3];
#define EN_ACC_TIME_TRIG 0
u8 ACC_FORWARD_BMP=0;
u8 ACC_FORWARD=0;
u8 en_bmp_avoid_wind=0;
float baro_compensate(float dT,float kup,float kdw,float vz,float lim)
{
	float z_sin;
	static float com_val,com_tar;
	
	z_sin = my_sqrt(1-my_pow(vz));
	
	//com_tar = (z_sin/0.44f) *lim;
	LPF_1_(2.0f,dT,((z_sin/0.44f) *lim),com_tar);
	com_tar = LIMIT(com_tar,0,lim);
	
	if(com_val<(com_tar-100))
	{
		com_val += 1000 *dT *kup;
	}
	else if(com_val>(com_tar+100))
	{
		com_val -= 1000 *dT *kdw;
	}
	return (com_val);
}


void navUkfQuatToMatrix(float *m, float *q, int normalize) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;

    // get the invert square length
    if (normalize)
	invs = 1.0f / (sqx + sqy + sqz + sqw);
    else
	invs = 1.0f;

    // rotation matrix is scaled by inverse square length
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0f * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0f * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0f * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0f * (tmp1 - tmp2) * invs;
}

void MatrixOpp(float a[], float result[])  
{  
    int const M=3;     //?????  
    int const N =2*M;  //???????  
    float b[M][N];    //????  
    int i,j,k;  
    for(i=0;i<M;i++)  
    {  
        for(j=0;j<M;j++)  
        {  
            b[i][j]=a[i*M+j];  
        }  
    }  
    /*****************????***********************/  
    for(i=0;i<M;i++)  
    {  
        for(j=M;j<N;j++)  
        {  
            if(i==(j-M))  
            {  
                b[i][j]=1;  
            }  
            else  
            {  
                b[i][j]=0;  
            }  
        }  
    }  
    /*****************????***********************/  
  
    /*****************????***********************/  
    for(i=0;i<M;i++)  
    {  
        if(b[i][i]==0)  
        {  
            for(k=i;k<M;k++)  
            {  
                if(b[k][k]!=0)  
                {  
                    for(int j=0;j<N;j++)  
                    {  
                        double temp;  
                        temp=b[i][j];  
                        b[i][j]=b[k][j];  
                        b[k][j]=temp;  
                    }  
                    break;  
                }  
            }  
           
        }  
        for(j=N-1;j>=i;j--)  
        {  
            b[i][j]/=b[i][i];  
        }  
        for(k=0;k<M;k++)  
        {  
            if(k!=i)  
            {  
                double temp=b[k][i];  
                for(j=0;j<N;j++)  
                {  
                    b[k][j]-=temp*b[i][j];  
                }  
            }  
        }  
    }  
    /*****************????***********************/  
  
    /*****************????***********************/  
    for(i=0;i<M;i++)  
    {  
        for(j=M;j<N;j++)  
        {  
            result[i*M+j-3]=b[i][j];  
        }  
    }  
    /*****************????***********************/  
  
} 
  

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

void navUkfRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}

#define H_HIST 40
void feed_acc_buf(float *in)
{
u8 i,j;	
float reg[3][H_HIST]={0};	
static u8 cnt;
 for(i=0;i<3;i++)
	for(j=0;j<H_HIST;j++)
   reg[i][j]=acc_body_buf[i][j];
	
 for(i=0;i<3;i++)
	for(j=1;j<H_HIST-1;j++)
   acc_body_buf[i][j]=reg[i][j-1];	
	
	acc_body_buf[0][0]=in[0];
	acc_body_buf[1][0]=in[1];
	acc_body_buf[2][0]=in[2];
}	



#define IIR_ORDER_ACC 10
static double b_IIR_acc[IIR_ORDER_ACC+1] ={
  0.00049945407823310042,
	0.0049945407823310042 ,
	0.022475433520489523 , 
	0.059934489387972065 , 
	0.10488535642895111  , 
	0.12586242771474132  , 
	0.10488535642895111   ,
	0.059934489387972065  ,
	0.022475433520489523  ,
	0.0049945407823310042 ,
	0.00049945407823310042,
};  //系数b
static double a_IIR_acc[IIR_ORDER_ACC+1] ={ 
 1            ,    
-1.9924014816014133  ,
3.0194828633553867  ,
-2.8185224264945168  , 	
2.0387206370625282   ,
-1.0545446210956813  ,
0.41444626875039958  ,
-0.11571862523682841  ,
0.022498509272218331 ,
-0.0026689123535761092,
0.0001487644521777628
};
static double InPut_IIR_acc[3][IIR_ORDER_ACC+1] = {0};
static double OutPut_IIR_acc[3][IIR_ORDER_ACC+1] = {0};
static double InPut_IIR_bmp[IIR_ORDER_ACC+1] = {0};
static double OutPut_IIR_bmp[IIR_ORDER_ACC+1] = {0};

float ALT_POS_BMP,ALT_VEL_BMP;
float ALT_POS_SONAR,ALT_VEL_SONAR,ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_BMP_EKF,ALT_VEL_BMP_EKF;
float ALT_POS_BMP_UKF_OLDX,ALT_VEL_BMP_UKF_OLDX,ALT_ACC_BMP_UKF_OLDX;

double P_baro[9]={1,0,0,0,1,0,0,0,1}; 
double X_ukf_baro[3];

double P_barob[16]={1,0,0,0,1,0,0,0,1}; 
double X_ukf_barob[4];
//-----------------KF  parameter------------------
float gh_bmp=0.5;
float k_fp_spd_bmp=10;
float gh_bmp1=0.01;
float k_fp_spd_bmp1=10;

float gh_sonar=0.005;
float k_fp_spd_sonar=10;
float ga=0.1;
float gwa=0.1;
//#if USE_M100_IMU
//float  r_baro_new[4]={0.015,0.05,0.03,3.5};
//#else
float  r_baro_new[4]={0.015,0.05,0.03,5};
//#endif
float  r_sonar_new[4]={0.036,0.056,0.026,2.5};


double P_kf_baro[9]={1,0,0,1,0,0,1,0,0}; 
double X_kf_baro[3];
double state_correct_baro[6];
double state_correct_sonar[6];
double P_kf_baro_bmp[9]={1,0,0,0,1,0,0,0,1}; 
double X_kf_baro_bmp[3];
//float r_baro_ukf[3]={1,1,1};float q_baro_ukf[3]={0.01,0.01,0.01};
float r_baro_ukf[4]={10,1,0.1,0.1};float q_baro_ukf[4]={0.001,0.001,0.001,0.001};

float dead_accz=0.00;
float acc_off_baro=0;
float acc_scale_bmp=1;//0.88;//0.88;
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
float acc_body[3],acc_body_buf[3][20];
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
u8 baro_flt_sel=3;
void ukf_baro_task1(float T)// 气压计加速度计融合
{
static u8 height_ctrl_mode_reg;	
static float off_bmp_sonar;		
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
		
	if(height_ctrl_mode_reg!=height_ctrl_mode&&height_ctrl_mode==1&&module.sonar==1&&ALT_POS_SONAR2<2)
	{
		off_bmp_sonar=-(float)(baro.relative_height)/1000.+ALT_POS_SONAR2;
	} 		
	#define BARO_AV_NUM_FU 100*1/2
	static float baro_av_arr_fu[BARO_AV_NUM_FU];
	static  u16 baro_av_cnt_fu;
	baro.h_origin=((float)(baro.relative_height)/1000.+off_bmp_sonar);
	switch(baro_flt_sel){
		case 0:Moving_Average1( (float)( baro.h_origin),baro_av_arr_fu,BARO_AV_NUM_FU, &baro_av_cnt_fu ,&baro.h_flt ); //单位mm/s
		break;
		case 1:baro.h_flt=firstOrderFilter((baro.h_origin) ,&firstOrderFilters[BARO_LOWPASS],T);
		break;
		case 2:baro.h_flt=IIR_I_Filter(baro.h_origin , InPut_IIR_bmp, OutPut_IIR_bmp, b_IIR_acc, IIR_ORDER_ACC+1, a_IIR_acc, IIR_ORDER_ACC+1);
		break;
		default:baro.h_flt=baro.h_origin;
	 break;
	}
	eso_h_spd.h0=eso_h_acc.h0=T;	
	OLDX_SMOOTH_IN_ESO(&eso_h_spd,baro.h_flt);
	baro.v_flt=eso_h_spd.v2;
	OLDX_SMOOTH_IN_ESO(&eso_h_acc,baro.v_flt);
	baro.acc_flt=eso_h_acc.v2;		
		
float baro_com_val;
#if EN_ATT_CAL_FC
baro_com_val = baro_compensate(dt,1.0f,1.0f,reference_vr_imd_down_fc[2],3500);
#else
baro_com_val = baro_compensate(dt,1.0f,1.0f,reference_vr_imd_down[2],3500);
#endif
float posz;
u8 input_flag=2;
if(mode_oldx.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1)){
#if USE_M100_IMU
	static u8 m100_connect_r;
	static float off_m100;	
	if(m100.m100_connect)	
	off_m100=	(float)(baroAlt-baro.relative_height)/1000.;
	posz=(float)(baro.relative_height)/1000.+off_m100;	
	input_flag=1;
	m100_connect_r=m100.m100_connect;
#else
	input_flag=1;
	posz=(float)(baro.h_flt);	
#endif
}
else
posz=LIMIT(ALT_POS_SONAR2,0,5);

static float temp_r;
u8 i,j;
float acc_temp1,temp;  
float accIn[3];
float acc_body_temp[3],acc_body_temp_flt[3];
 		accIn[0] =(float) mpu6050_fc.Acc.x/4096.*9.8;
    accIn[1] =(float) mpu6050_fc.Acc.y/4096.*9.8;
		accIn[2] =(float) mpu6050_fc.Acc.z/4096.*9.8;
    body_to_NEZ(acc_body_temp_flt, accIn, ref_q_imd_down_fc);


    acc_z_att_corr=-my_sqrt(pow(my_sqrt(pow(sin(Pit_fc*0.0173),2)+pow(sin(Rol_fc*0.0173),2)),2));

		static float wz_acc ;
		static u16 ekf_init_cnt;
	
		
	
		acc_body_temp[0] = IIR_I_Filter(LIMIT(acc_body_temp_flt[0],-6.6,6.6) , InPut_IIR_acc[0], OutPut_IIR_acc[0], b_IIR_acc, IIR_ORDER_ACC+1, a_IIR_acc, IIR_ORDER_ACC+1);
		acc_body_temp[1] = IIR_I_Filter(LIMIT(acc_body_temp_flt[1],-6.6,6.6) , InPut_IIR_acc[1], OutPut_IIR_acc[1], b_IIR_acc, IIR_ORDER_ACC+1, a_IIR_acc, IIR_ORDER_ACC+1);
		acc_body_temp[2] = IIR_I_Filter(LIMIT(acc_body_temp_flt[2]-9.78,-6.6,6.6) , InPut_IIR_acc[2], OutPut_IIR_acc[2], b_IIR_acc, IIR_ORDER_ACC+1, a_IIR_acc, IIR_ORDER_ACC+1);
					 
		
		
		if(NS==0)
	  acc_off_baro=0;
		else if((!fly_ready&&NS==2))		
		{
		acc_off_baro += ( 1 / ( 1 + 1 / ( 3*0.6f *3.14f *dt ) ) ) *(acc_body_temp[2]- acc_off_baro) ;
		acc_off_baro=LIMIT(acc_off_baro,-3,3);
		}
		
    acc_body[2]=(acc_body_temp[2]-acc_off_baro*1)*acc_scale_bmp-LIMIT(ALT_ACC_BMP_UKF_OLDX,-1,1)*0;
		
	//	if(ekf_init_cnt++>256 && fabs(acc_body[2])<3)
		{sys_init.baro_ekf=1;}
		
		feed_acc_buf(acc_body);
		acc_bmp=acc_body[2];
		#if EN_ACC_TIME_TRIG
		if(mode_oldx.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))
		ACC_FORWARD=ACC_FORWARD_BMP;
		else
		ACC_FORWARD=0;	
		acc_bmp=acc_body_buf[2][ACC_FORWARD];
		#endif

				
		if(height_ctrl_mode_reg!=height_ctrl_mode)
		{
			X_kf_baro[0]=posz;
		} 	
		height_ctrl_mode_reg=height_ctrl_mode;
		
	float gh_use,k_fp_spd_use;
	if(mode_oldx.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))
  {gh_use=gh_bmp;k_fp_spd_use=k_fp_spd_bmp;}
  else 
  {gh_use=gh_sonar;k_fp_spd_use=k_fp_spd_sonar;}	
	
	static u8 wind_error=0;
	//#define BARO_KF2 
	#define BARO_KF	

	#if defined(BARO_KF2) //KF with limit bias-----------------------------------------------------------
	#if EN_ACC_TIME_TRIG
	k_fp_spd_use=0;
	#endif
	u8 flag_sensor[3]={1,0,1};
	float temp_R;
//	
//	if((acc_z_acc[2]>1||fabs(ALT_POS_BMP_UKF_OLDX-posz)>0.6)&&(mode_oldx.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))&&en_bmp_avoid_wind)
//	{temp_R=1;wind_error=1;}
//	else
	 temp_R=r_baro_new[3];
	float Z_kf[3]={posz+LIMIT(X_kf_baro[1],-1,1)*T*k_fp_spd_use*1,0,acc_bmp};
	if(mode_oldx.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))
  OLDX_KF2(Z_kf,temp_R,r_baro_new,flag_sensor,X_kf_baro,state_correct_baro,T);
	else
	OLDX_KF2(Z_kf,r_sonar_new[3],r_sonar_new,flag_sensor,X_kf_baro,state_correct_baro,T);
  
	#if EN_ACC_TIME_TRIG
	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0]+(ACC_FORWARD)*T*X_kf_baro[1]+1/2*pow((ACC_FORWARD)*T,2)*(acc_bmp+X_kf_baro[2]);
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1]+(ACC_FORWARD)*T*(acc_bmp+X_kf_baro[2]);
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];	
	#else
	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0];
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1];
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];	
	#endif
	#elif  defined(BARO_KF) //KF with bias-----------------------------------------------------------------------
	#if EN_ACC_TIME_TRIG
	k_fp_spd_use=0;
	#endif
	double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

	double B[3]={T*T/2,T,0}; 
	double H[9]={
			 1,0,0,
			 0,0,0,
			 0,0,0}; 
//	if((acc_z_acc[2]>1||fabs(ALT_POS_BMP_UKF_OLDX-posz)>0.6)&&(mode_oldx.height_safe||height_ctrl_mode==1||(NS==0&&test_bmp==1))&&en_bmp_avoid_wind)
//	{gh_use*=2000;wind_error=1;}
	
//	if(wind_error)
//     if(fabs(ALT_POS_BMP_UKF_OLDX-posz)<0.5)
//       wind_error=0;
//				else	
//	   gh_use+=fabs(ALT_POS_BMP_UKF_OLDX-posz)/0.5*1000;
	double Z_kf[3]={posz+LIMIT(ALT_VEL_BMP_UKF_OLDX,-1,1)*T*k_fp_spd_use*1,0,0};
	if(sys_init.baro_ekf)
	//kf_oldx( X_kf_baro,  P_kf_baro,  Z_kf,  acc_bmp, gh_use,  ga,  gwa,T);//for spd
  KF_OLDX_NAV( X_kf_baro,  P_kf_baro,  Z_kf,  acc_bmp, A,  B,  H,  ga,  gwa, gh_use,  0.001,  T);
	#if EN_ACC_TIME_TRIG
	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0]+(ACC_FORWARD-1)*T*X_kf_baro[1]+1/2*pow((ACC_FORWARD-1)*T,2)*X_kf_baro[2];
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1]+(ACC_FORWARD-1)*T*X_kf_baro[2];
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];	
	#else
	ALT_POS_BMP_UKF_OLDX+=ALT_VEL_BMP_UKF_OLDX*T;//X_kf_baro[0];
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1];
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];	
	#endif
	
	static float acc_reg[2];
	acc_z_acc[0]=(acc_bmp-acc_reg[0])/T;
	acc_z_acc[1]=(ALT_ACC_BMP_UKF_OLDX-acc_reg[1])/T;
	acc_reg[0]=acc_bmp;
	acc_reg[1]=ALT_ACC_BMP_UKF_OLDX;
	acc_z_acc[2]=fabs(acc_z_acc[0]-acc_z_acc[1]);
	
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
	
//	if(fabs(acc_est)>10||fabs(ALT_VEL_BMP_UKF_OLDX)>4)	
//  {ero.baro_ekf=1;ero.baro_ekf_cnt++;}
//	#if !defined(BARO_KF2)
//	if((fabs(ALT_POS_SONAR2-ALT_POS_BMP_UKF_OLDX)>0.36||(fabs(acc_est)>10||fabs(ALT_VEL_BMP_UKF_OLDX)>4))&&input_flag==2)	
//  {ero.baro_ekf=1;ero.baro_ekf_cnt++;}
//  #endif
//	float DCM_Q[9];
//	navUkfQuatToMatrix(DCM_Q,ref_q_imd_down_fc,0);
//	//float acc_bias_NEZ[3]={0,0,ALT_ACC_BMP_UKF_OLDX};
//	//navUkfRotateVecByRevMatrix(acc_bias,acc_bias_NEZ,DCM_Q);
//	  float corr_baro = flag_ero*( posz- ALT_POS_BMP_UKF_OLDX);
//		//accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;
//	  accel_bias_corr[2]= ALT_ACC_BMP_UKF_OLDX;
//    float R_control_now1[9];
//		R_control_now1[0]=R_control_now[0][0];R_control_now1[3]=R_control_now[0][1];R_control_now1[6]=R_control_now[0][2];
//		R_control_now1[1]=R_control_now[1][0];R_control_now1[4]=R_control_now[1][1];R_control_now1[7]=R_control_now[1][2];
//		R_control_now1[2]=R_control_now[2][0];R_control_now1[5]=R_control_now[2][1];R_control_now1[8]=R_control_now[2][2];
//		/* transform error vector from NED frame to body frame */
//		for (int i = 0; i < 3; i++) {
//			float c = 0.0f;

//			for (int j = 0; j < 3; j++) {
//				c += PX4_R(R_control_now1, j, i)*accel_bias_corr[j];
//			}

//			if (isfinite(c)) {
//				acc_bias[i] += c * w_acc_bias * T;
//			}
//		}
}