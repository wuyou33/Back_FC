#include "include.h"
#include "rc.h"
#include "imu.h"
#include "ctrl.h"
#include "filter.h"
#include "alt_fushion.h"
CIRCLE circle,track;
MARKER marker;
float nav_circle[2],nav_circle_last[2];
/*
        y add (0~256)  1
        |
        |


        -------- x decrease (0~320)  0


*/
float circle_check=0.01;
float circle_lfp=1;

void circle_control(float T)//对圆自动降落 未使用
{
}

float data_d(float in,float *reg,float T,float max)//微分函数
{
float temp;
	temp=(in-*reg)/T;
  *reg=in;
return LIMIT(temp,-max,max);
}

float out_back_step[3];
float k_back_step=1;//1~2.5;
void backstep_pos(float T,float mess,float exp_x,float exp_y)//反步法控制器  未测试
{
float alfa[6]={1};	
alfa[0]=alfa[1]=alfa[2]=alfa[3]=alfa[4]=alfa[5]=k_back_step;
float x_now=POS_UKF_X;
float y_now=POS_UKF_Y;	
float z_now=ALT_POS_SONAR2;
float exp_z=z_now;
static float r_x_now,r_exp_x,r_exp_y,r_y_now,r_exp_z,r_z_now;
float d_exp_x=data_d(exp_x,&r_exp_x,T,10);
float d_exp_y=data_d(exp_y,&r_exp_y,T,10);
float d_exp_z=data_d(exp_z,&r_exp_z,T,10);
float q1=exp_x-POS_UKF_X;
float q2=d_exp_x-data_d(x_now,&r_x_now,T,10)+alfa[0]*q1;
float q3=exp_y-y_now;
float q4=d_exp_y-data_d(y_now,&r_y_now,T,10)+alfa[2]*q3;
float q5=exp_z-z_now;
float q6=d_exp_z-data_d(z_now,&r_z_now,T,10)+alfa[4]*q5;

float fr=mess*((1-pow(alfa[2],2))*q3+(alfa[2]+alfa[3])*q4+d_exp_y+9.8)/(cos(Pit_fc*0.0173)*cos(Rol_fc*0.0173));
	if(fr==0)
		fr=0.00001;
float u1=mess*((1-pow(alfa[0],2))*q1+(alfa[0]+alfa[1])*q2+d_exp_x)/(fr);
float u2=mess*((1-pow(alfa[4],2))*q5+(alfa[4]+alfa[5])*q6+d_exp_z)/(fr);

out_back_step[PITr]=u1*sin(Yaw_fc*0.0173)+u2*cos(Yaw_fc*0.0173);
out_back_step[ROLr]=(-u1*cos(Yaw_fc*0.0173)+u2*sin(Yaw_fc*0.0173))/cos(Pit_fc*0.0173);
}

#define NAV_ANGLE_MAX 0.8*MAX_CTRL_ANGLE
float nav[2];
float GPS_angle[2];
float  target_position[2];
float  now_position[2];
float actual_speed[2];
float tar_speed[2],tar_speed_avoidc[2];
float d_flow_watch[2];
float  integrator[2];
float k_break=0.85,k_flt_break=0.03;
u8 cnt_sb_sample_max=20;
float scale_flow_rc=0.05;
float r_circle=0.25;
float d_angle=0.4;
float circle_angle;

#define Mark_Size 0.18
#define Mark_Dis  0.36
#define NUM_HAN 10
#define NUM_LIE 10
#define WAY_POINT_DEAD 0.1

float exp_center_cycle[5]={1.50,1.50,0.78,0,20};
u8 line_test_num[2]={12,34};
u8 tan_test_num[4]={12,32,34,14};
void Nav_pos_set_test(u8 mode_in,float T)//轨迹规划
{
u8 i;	
static float init_pos[2];	
static u8 mode_circle_reg,line_flag=0,tangle_flag=0;
float exp_ling[2][2];
float exp_tan[4][2];	
if((mode_in==2&&mode_circle_reg!=2)||mode.flow_hold_position<=1)
{
 init_pos[X]=POS_UKF_X;
 init_pos[Y]=POS_UKF_Y; 
 exp_center_cycle[3]=0;
}
mode_circle_reg=mode_in;	

switch(mode_in){
	case 2://cycle
		  exp_center_cycle[3]+=T*exp_center_cycle[4];
			if(exp_center_cycle[3]<0)exp_center_cycle[3]=360;
			if(exp_center_cycle[3]>360)exp_center_cycle[3]=0;
	        nav_pos_ctrl[X].exp=cos(exp_center_cycle[3]*0.0173)*exp_center_cycle[2]-exp_center_cycle[2]*0+init_pos[X];
					nav_pos_ctrl[Y].exp=sin(exp_center_cycle[3]*0.0173)*exp_center_cycle[2]-exp_center_cycle[2]*0+init_pos[Y];
	break;
	case 1://line
  	for(i=0;i<2;i++)
 {
	  exp_tan[i][X] = (line_test_num[i]%NUM_LIE-1+(line_test_num[i]%NUM_LIE==0?NUM_LIE:0))*Mark_Dis;
	  exp_tan[i][Y] = -(line_test_num[i]/NUM_LIE-(line_test_num[i]%NUM_LIE==0?1:0))*Mark_Dis; 
 }	
	
	nav_pos_ctrl[X].exp=exp_tan[tangle_flag][X];
	nav_pos_ctrl[Y].exp=exp_tan[tangle_flag][Y];
    
	if(fabs(nav_pos_ctrl[X].exp - nav_pos_ctrl[X].now)<WAY_POINT_DEAD&&fabs(nav_pos_ctrl[Y].exp - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD)
	 { if(tangle_flag<1)tangle_flag+=1;
	   else tangle_flag=0;}
	
	break;
	case 3://way_point
	for(i=0;i<4;i++)
 {
	  exp_tan[i][X] = (tan_test_num[i]%NUM_LIE-1+(tan_test_num[i]%NUM_LIE==0?NUM_LIE:0))*Mark_Dis;
	  exp_tan[i][Y] = -(tan_test_num[i]/NUM_LIE-(tan_test_num[i]%NUM_LIE==0?1:0))*Mark_Dis; 
 }	
	
	nav_pos_ctrl[X].exp=exp_tan[tangle_flag][X];
	nav_pos_ctrl[Y].exp=exp_tan[tangle_flag][Y];
    
	if(fabs(nav_pos_ctrl[X].exp - nav_pos_ctrl[X].now)<WAY_POINT_DEAD&&fabs(nav_pos_ctrl[Y].exp - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD)
	 { if(tangle_flag<3)tangle_flag+=1;
	   else tangle_flag=0;}
	break;
	
	default:
	break;

}
}
u8 pos_exp_test=0;
float yaw_qr_off;
float out_timer_nav,in_timer_nav;
float acc_temp[3];
_pos_pid nav_pos_pid;
_pos_pid nav_spd_pid;
_pos_pid nav_acc_pid;
_pos_control nav_pos_ctrl[2];
_pos_control nav_spd_ctrl[2];
_pos_control nav_acc_ctrl[2];
void reset_nav_pos(u8 sel)
{
if(sel==Y)	
nav_pos_ctrl[Y].exp=POS_UKF_Y;//mm
if(sel==X)
nav_pos_ctrl[X].exp=POS_UKF_X;//mm
}


void Positon_control(float T)// 位置控制 
{ u8 i;
	static u8 cnt[2],init;
	if(!init){init=1;
		//pos
		nav_pos_ctrl[X].mode=2;
		nav_pos_pid.kp=0.45;
		nav_pos_pid.ki=0.00;
		nav_pos_pid.kd=0.0;
		nav_pos_pid.dead=0.001;
		//adrc
		eso_pos[X].b0=eso_pos[Y].b0=4.5;
		eso_pos[X].err_limit=eso_pos[Y].err_limit=8000;
		eso_pos[X].eso_dead=eso_pos[Y].eso_dead=nav_pos_pid.dead*1000;
	  //spd	
		nav_spd_pid.f_kp=0.2;
		nav_spd_pid.kp=0.125;
		nav_spd_pid.ki=0.150;
		nav_spd_pid.kd=0.15;
		nav_spd_pid.flt_nav_kd=0.0;
		nav_spd_pid.dead=0.005*1000;
		nav_spd_pid.flt_nav=0.75;//决定刹车手感
		//acc
		nav_acc_pid.f_kp=0.2;
		nav_acc_pid.kp=0.15;
		nav_acc_pid.ki=0.00;
		nav_acc_pid.kd=0.0;	
		nav_acc_pid.dead=0.01*1000;
		nav_acc_pid.flt_nav_kd=15;
	}
	static u8 pos_reset_state[2];
	static u16 pos_reset_cnt[2];
	if(NS==0||mode.rc_control_flow_pos_sel||Thr_Low)
	Nav_pos_set_test(mode.rc_control_flow_pos_sel,T);
	else if(smart.rc.POS_MODE==0){
	for(i=0;i<2;i++){
			switch(pos_reset_state[i])
			{
				case 0:
				 if(fabs(CH_filter[i])>25)
				 {pos_reset_state[i]=0;pos_reset_cnt[i]=0;
				  reset_nav_pos(X);reset_nav_pos(Y);
				 }
				break;
				case 1:
				 reset_nav_pos(i);
				 float angle_check;
				 if(i==Y)angle_check=Pit_fc;else angle_check=Rol_fc;
				 if(fabs(CH_filter[i])<25)
				   pos_reset_cnt[i]++;
				 else
					 pos_reset_cnt[i]=0;
				 
				 if((fabs(CH_filter[i])<25&&fabs(angle_check)<5)||pos_reset_cnt[i]>4.0/LIMIT(T,0,0.001))
					 pos_reset_state[i]=0; 
				break;
			}	
  }
	if(!pos_exp_test){
	if(ALT_POS_SONAR2<SONAR_HEIGHT*1.45||!fly_ready||mode.flow_hold_position==0)
	{reset_nav_pos(Y);reset_nav_pos(X);}
	}
	
	if((!fly_ready||Thr_Low)&&NS==2)
	{ 
		reset_nav_pos(Y);reset_nav_pos(X);
	  nav_pos_ctrl[X].err_i=nav_pos_ctrl[Y].err_i=nav_acc_ctrl[X].err_i=nav_acc_ctrl[Y].err_i=nav_spd_ctrl[X].err_i=nav_spd_ctrl[Y].err_i=0;
	}
 }
	
	static u8 mode_flow_hold_position_reg;
	if(mode_flow_hold_position_reg!=mode.flow_hold_position)
	{reset_nav_pos(Y); reset_nav_pos(X);}
	mode_flow_hold_position_reg=mode.flow_hold_position;
	
	if(mode.flow_f_use_ukfm==1||mode.flow_f_use_ukfm==2)
		nav_pos_ctrl[X].mode=2;//golable mode
	else if(mode.flow_f_use_ukfm==0)
		nav_pos_ctrl[X].mode=1;
	
//	if(mode.flow_hold_position==2)
//		nav_spd_pid.flt_nav=1;
//	else
//		nav_spd_pid.flt_nav=0.3;
/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	

	   
head  |    1 PIT y-   90d in marker
			| 
		   _____  0 ROL x+

*/
  float pos[2];
	int spd[2],acc[2];
	float a_br[3];	
	static float acc_flt[2];
	#if EN_ATT_CAL_FC
	a_br[0] =(float) mpu6050_fc.Acc.x/4096.;//16438.;
	a_br[1] =(float) mpu6050_fc.Acc.y/4096.;//16438.;
	a_br[2] =(float) mpu6050_fc.Acc.z/4096.;//16438.;
  #else
	a_br[0] =(float) mpu6050.Acc.x/4096.;//16438.;
	a_br[1] =(float) mpu6050.Acc.y/4096.;//16438.;
	a_br[2] =(float) mpu6050.Acc.z/4096.;//16438.;
	#endif
	#if EN_ATT_CAL_FC
	acc_temp[0] = a_br[1]*reference_vr_imd_down_fc[2]  - a_br[2]*reference_vr_imd_down_fc[1] ;
	acc_temp[1] = a_br[2]*reference_vr_imd_down_fc[0]  - a_br[0]*reference_vr_imd_down_fc[2] ;
	#else
	acc_temp[0] = a_br[1]*reference_vr_imd_down[2]  - a_br[2]*reference_vr_imd_down[1] ;
	acc_temp[1] = a_br[2]*reference_vr_imd_down[0]  - a_br[0]*reference_vr_imd_down[2] ;
	#endif
	float k_acc_flt;
	if(mode.test4)
	k_acc_flt=nav_acc_pid.flt_nav_kd;
	else
	k_acc_flt=15;	
	//acc_flt[0] += ( 1 / ( 1 + 1 / ( k_acc_flt *3.14f *T ) ) ) *my_deathzoom1( (-acc_temp[0] - acc_flt[0] ),0);
	//acc_flt[1] += ( 1 / ( 1 + 1 / ( k_acc_flt *3.14f *T ) ) ) *my_deathzoom1( (-acc_temp[1] - acc_flt[1] ),0);
	acc_flt[0]=firstOrderFilter(-acc_temp[0] ,&firstOrderFilters[ACC_LOWPASS_X],T);
	acc_flt[1]=firstOrderFilter(-acc_temp[1] ,&firstOrderFilters[ACC_LOWPASS_Y],T);
	float b[3] = {0.8122  ,  1.6244  ,  0.8122};
	float a[3] = {1.0000  ,  1.5888  ,  0.6600};
	float xBuf1[3];
	float yBuf1[3];
	float xBuf2[3];
	float yBuf2[3];
	
	
//输入数据	
	pos[Y]=POS_UKF_Y;//mm
  pos[X]=POS_UKF_X;//mm
	
	spd[Y]=VEL_UKF_Y*1000;//mm
  spd[X]=VEL_UKF_X*1000;//mm
	acc_body[Y]=acc[Y]=acc_flt[Y]*9800;//IIR_LP(acc_flt[Y]*9800,xBuf1,yBuf1,a,b,2);
  acc_body[X]=acc[X]=acc_flt[X]*9800;//IIR_LP(acc_flt[X]*9800,xBuf2,yBuf2,a,b,2);

		
//位置
	//smart
	if((smart.rc.POS_MODE==SMART_MODE_POS&&fabs(smart.spd.x)>0)||(smart.rc.POS_MODE==SMART_MODE_RC&&fabs(smart.rc.PITCH-1500)>25)||(smart.rc.POS_MODE==SMART_MODE_RC&&fabs(smart.rc.ROLL-1500)>25))
	nav_pos_ctrl[X].exp=smart.pos.x;
	if((smart.rc.POS_MODE==SMART_MODE_POS&&fabs(smart.spd.y)>0)||(smart.rc.POS_MODE==SMART_MODE_RC&&fabs(smart.rc.PITCH-1500)>25)||(smart.rc.POS_MODE==SMART_MODE_RC&&fabs(smart.rc.ROLL-1500)>25))
	nav_pos_ctrl[Y].exp=smart.pos.y;
	
	//
	if(cnt[0]++>0){cnt[0]=0;
	float temp1;
  temp1=(float)Get_Cycle_T(GET_T_OUT_NAV)/1000000.;
	
	if(temp1>0.001)
		out_timer_nav=temp1;
	else
		out_timer_nav=0.04;
	
	for (i=0;i<2;i++){ 
		nav_pos_ctrl[i].now=pos[i];
	  OLDX_POS_CONTROL_ESO(&eso_pos[i],nav_pos_ctrl[i].exp*1000,nav_pos_ctrl[i].now*1000,eso_pos[i].u,out_timer_nav,1.5*1000,nav_pos_pid.kp,thr_view );//速度环自抗扰控制	
		nav_pos_ctrl[i].err = ( nav_pos_pid.kp*LIMIT(my_deathzoom1(nav_pos_ctrl[i].exp - nav_pos_ctrl[i].now,nav_pos_pid.dead),-1.5,1.5)*1000 );
    if(eso_pos[i].b0==0&&nav_pos_pid.ki>0){
		nav_pos_ctrl[i].err_i += nav_pos_pid.ki *nav_pos_ctrl[i].err *out_timer_nav;
		nav_pos_ctrl[i].err_i = LIMIT(nav_pos_ctrl[i].err_i,-Thr_Weight *NAV_POS_INT,Thr_Weight *NAV_POS_INT);
		}
		else
		nav_pos_ctrl[i].err_i=0;
		nav_pos_ctrl[i].err_d =  nav_pos_pid.kd *( 0.6f *(-(float)spd[i]*out_timer_nav) + 0.4f *(nav_pos_ctrl[i].err - nav_pos_ctrl[i].err_old) );
if(eso_pos[i].b0==0)
		nav_pos_ctrl[i].pid_out = nav_pos_ctrl[i].err +nav_pos_ctrl[i].err_i + nav_pos_ctrl[i].err_d;
else
	  nav_pos_ctrl[i].pid_out = nav_pos_ctrl[i].err_d +eso_pos[i].u;

		nav_pos_ctrl[i].pid_out = LIMIT(nav_pos_ctrl[i].pid_out,-1.5*1000,1.5*1000);//m/s
		nav_pos_ctrl[i].err_old = nav_pos_ctrl[i].err;
		}
	}
	
	
	if(circle.check&&circle.connect)
	yaw_qr_off=circle.yaw-Yaw_fc;
  else if(circle.connect==0)
	yaw_qr_off=0;
	#if EN_ATT_CAL_FC
	float Yaw_qr=To_180_degrees(Yaw_fc+yaw_qr_off);
	#else
	float Yaw_qr=To_180_degrees(Yaw+yaw_qr_off);
	#endif
	
	float temp_pos_out[2];
	if(nav_pos_ctrl[X].mode==2){//global  Yaw from IMU
	temp_pos_out[Y]= nav_pos_ctrl[North].pid_out*cos(Yaw_qr*0.0173)+nav_pos_ctrl[East].pid_out*sin(Yaw_qr*0.0173); 
	temp_pos_out[X]=-nav_pos_ctrl[North].pid_out*sin(Yaw_qr*0.0173)+nav_pos_ctrl[East].pid_out*cos(Yaw_qr*0.0173);
	}
	else
	{
	temp_pos_out[Y]=nav_pos_ctrl[Y].pid_out;
	temp_pos_out[X]=nav_pos_ctrl[X].pid_out;	
	}		
	
 if(smart.rc.POS_MODE==SMART_MODE_SPD)//only for smart_spd
	 {
		 if(smart.spd.y==0)
		 nav_spd_ctrl[Y].exp=temp_pos_out[Y]; 
		 if(smart.spd.x==0)
		 nav_spd_ctrl[X].exp=temp_pos_out[X]; 
	 } 
	 else
	 {
	 nav_spd_ctrl[Y].exp=temp_pos_out[Y];
	 nav_spd_ctrl[X].exp=temp_pos_out[X];
	 } 
	
	
  if(mode.flow_hold_position!=2){//速度阶越测试
	 nav_spd_ctrl[Y].exp*=0.0;
	 nav_spd_ctrl[X].exp*=0.0;}
	static u8 state_tune_spd;
	static u8 flag_way;
	static u16 cnt_s1;
	switch(state_tune_spd){
	case 0:	
	if(mode.trig_flow_spd)
	{state_tune_spd=1;cnt_s1=0;flag_way=!flag_way;}
	break;
	case 1:
	if(mode.trig_flow_spd)
	{	if(flag_way)
	nav_spd_ctrl[X].exp=300;
	else
	nav_spd_ctrl[X].exp=-300;		
	}
	else
	state_tune_spd=0;	
	if(cnt_s1++>3.0/T)
	{cnt_s1=0;state_tune_spd=2;}
	break;
	case 2:
	nav_spd_ctrl[X].exp=0;			
	if(cnt_s1++>1.5/T)	
	state_tune_spd=0;
	if(!mode.trig_flow_spd)
	state_tune_spd=0;
	break;
	}
//速度环	
	float EXP_LPF_TMP_SPD[2],EXP_LPF_TMP_ACC[2];;
	static float spd_old[2],acc_old[2];
	float temp;
  temp=(float)Get_Cycle_T(GET_T_IN_NAV)/1000000.;
	
	if(temp>0.001)
		in_timer_nav=temp;
	else
		in_timer_nav=0.02;
	
	if(fabs(CH_filter[0])>25||fabs(CH_filter[1])>25)
	{ nav_spd_ctrl[X].exp=nav_spd_ctrl[X].now;
	  nav_spd_ctrl[Y].exp=nav_spd_ctrl[Y].now;
	}
	
	for (i=0;i<2;i++){	
	nav_spd_ctrl[i].now=(spd[i]);
	nav_spd_ctrl[i].exp = LIMIT(nav_spd_ctrl[i].exp, -MAX_CTRL_POS_SPEED,MAX_CTRL_POS_SPEED );
	nav_spd_ctrl[i].damp = ( nav_spd_ctrl[i].now - spd_old[i]) *( 0.02f/in_timer_nav );
	nav_spd_ctrl[i].err =  my_deathzoom1( nav_spd_ctrl[i].exp - nav_spd_ctrl[i].now ,nav_spd_pid.dead);
	nav_spd_ctrl[i].err_weight = (float)ABS(nav_spd_ctrl[i].err)/MAX_CTRL_POS_SPEED;
	if(!mode.test4)
		nav_spd_ctrl[i].err_d = ( nav_spd_pid.kd  *( -10 *nav_spd_ctrl[i].damp*(1-nav_spd_pid.flt_nav_kd)+my_deathzoom1(-acc[i] ,50)*(nav_spd_pid.flt_nav_kd)) *( 0.02f/in_timer_nav ) );
	else 
		nav_spd_ctrl[i].err_d=0;
	
	if((fabs(nav_spd_ctrl[i].err)<eso_pos_spd[i].eso_dead||eso_pos_spd[i].b0==0||1)&&nav_spd_pid.ki>0){
	nav_spd_ctrl[i].err_i += nav_spd_pid.ki  *(nav_spd_ctrl[i].err) *in_timer_nav;
	nav_spd_ctrl[i].err_i = LIMIT( nav_spd_ctrl[i].err_i, -NAV_SPD_INT,NAV_SPD_INT );
	}
	else
	nav_spd_ctrl[i].err_i=0;	

	if(eso_pos_spd[i].b0==0||1){
	nav_spd_ctrl[i].pid_out  =  ( nav_spd_pid.f_kp *LIMIT((0.45f + 0.55f*nav_spd_ctrl[i].err_weight),0,1)*nav_spd_ctrl[i].exp + 
	(1 - nav_spd_pid.f_kp ) *nav_spd_pid.kp  *( nav_spd_ctrl[i].err + nav_spd_ctrl[i].err_d + nav_spd_ctrl[i].err_i ) );}
	else
	{
	nav_spd_ctrl[i].pid_out  =  ( nav_spd_pid.f_kp *LIMIT((0.45f + 0.55f*nav_spd_ctrl[i].err_weight),0,1)*nav_spd_ctrl[i].exp + 
	(1 - nav_spd_pid.f_kp ) *(  nav_spd_pid.kp  *nav_spd_ctrl[i].err_d +eso_pos_spd[i].u/10.) );}
  if(mode.test4)
	nav_spd_ctrl[i].pid_out =(float)(nav_spd_ctrl[i].pid_out);	\
  else
	nav_spd_ctrl[i].pid_out =(float)LIMIT((nav_spd_ctrl[i].pid_out),-250,250)/10.;	\
	nav_spd_ctrl[i].err_old= nav_spd_ctrl[i].err;
	spd_old[i] = nav_spd_ctrl[i].now  ;
	
	
// acc loop	
		if(mode.test4)
		{ 
			nav_acc_ctrl[i].now=acc_body[i];
			nav_acc_ctrl[i].exp=nav_spd_ctrl[i].pid_out*10*(8./12.);
			nav_acc_ctrl[i].exp = LIMIT(nav_acc_ctrl[i].exp, -MAX_CTRL_POS_ACC,MAX_CTRL_POS_ACC );
			nav_acc_ctrl[i].damp = ( nav_acc_ctrl[i].now - acc_old[i]) *( 0.02f/in_timer_nav );
			nav_acc_ctrl[i].err =  my_deathzoom1(( nav_acc_ctrl[i].exp - nav_acc_ctrl[i].now ),nav_acc_pid.dead) *(1000.0f/MAX_CTRL_POS_ACC);
			nav_acc_ctrl[i].err_d = ( nav_acc_pid.kd  *( -10 *nav_acc_ctrl[i].damp) *( 0.02f/in_timer_nav ) );
			if((fabs(nav_acc_ctrl[i].err)<eso_pos_spd[i].eso_dead||eso_pos_spd[i].b0==0)&&nav_acc_pid.ki>0){
			nav_acc_ctrl[i].err_i += nav_acc_pid.ki  *(nav_acc_ctrl[i].err) *in_timer_nav;
			nav_acc_ctrl[i].err_i = LIMIT( nav_acc_ctrl[i].err_i, -NAV_ACC_INT,NAV_ACC_INT );
			}
			else
			nav_acc_ctrl[i].err_i=0;	
			
			if(eso_pos_spd[i].b0==0||1){
			nav_acc_ctrl[i].pid_out  =  ( nav_acc_pid.f_kp*nav_acc_ctrl[i].exp + 
			(1 - nav_acc_pid.f_kp ) *nav_acc_pid.kp  *( nav_acc_ctrl[i].err + nav_acc_ctrl[i].err_d + nav_acc_ctrl[i].err_i ) );}
			else
			{	nav_acc_ctrl[i].pid_out  =  (nav_acc_pid.f_kp*nav_acc_ctrl[i].exp + 
			(1 - nav_acc_pid.f_kp ) *(  nav_acc_pid.kp  *nav_acc_ctrl[i].err_d +eso_pos_spd[i].u) );}

			nav_acc_ctrl[i].pid_out =(float)LIMIT((float)(nav_acc_ctrl[i].pid_out)/10.,-25,25);	
			nav_acc_ctrl[i].err_old= nav_acc_ctrl[i].err;
			acc_old[i] = nav_acc_ctrl[i].now  ;
		}
  }
		
	float flt_use[2];
	static u8 move[2]={0};
	if(mode.sb_smooth)
	{
			switch(move[Y])	{
			case 0:
			flt_use[Y]=nav_spd_pid.flt_nav;	
			if(fabs(CH_filter[PITr])>25&&move[Y]==0)
			move[Y]=1;
			break;
			case 1:
			flt_use[Y]=0.2;
			if(move[Y]&&fabs(CH_filter[PITr])==0)
			{ move[Y]=2;}
			break;
			case 2:
			if(move[Y]==2&&(fabs(Pit_fc)<2||fabs(CH_filter[PITr])>25))
			{move[Y]=0;}
		  break;
		 }
		switch(move[X])	{
			case 0:
			flt_use[X]=nav_spd_pid.flt_nav;	
			if(fabs(CH_filter[ROLr])>25&&move[X]==0)
			move[X]=1;
			break;
			case 1:
			flt_use[X]=0.2;
			if(move[X]&&fabs(CH_filter[ROLr])==0)
			{ move[X]=2;}
			break;
			case 2:
			if(move[X]==2&&(fabs(Rol_fc)<2||fabs(CH_filter[ROLr])>25))
			{move[X]=0;}
		  break;
		 }
	}
	else
	flt_use[Y]=flt_use[X]=nav_spd_pid.flt_nav;
	
	if(mode.test4){
  nav[PITr]=flt_use[Y]*nav_acc_ctrl[Y].pid_out+(1-flt_use[Y])*nav[PITr];
	nav[ROLr]=flt_use[X]*nav_acc_ctrl[X].pid_out+(1-flt_use[X])*nav[ROLr];
	}
	else
	{
  nav[PITr]=flt_use[Y]*nav_spd_ctrl[Y].pid_out+(1-flt_use[Y])*nav[PITr];
	nav[ROLr]=flt_use[X]*nav_spd_ctrl[X].pid_out+(1-flt_use[X])*nav[ROLr];
	}	

}


//--------------------------------------自动起飞降落 视觉导航状态机
u8 mode_change;
u8 state_v;
u8 force_pass;
static u16 cnt[10]={0};
u16 AUTO_UP_CUARVE[]={1600,1660,1660,1655,1650,1650,1650,1650,1650};
u16 AUTO_DOWN_CUARVE[]={1500,1500-50,1500-150,1500-150,1500-200,1500-200};
u16 AUTO_DOWN_CUARVE1[]={1500-150,1500-150,1500-100,1500-100,1500-80,1500-80};

void AUTO_LAND_FLYUP(float T)
{ 
	switch(state_v)
	{
		case SG_LOW_CHECK:
			if(mode.auto_fly_up&&CH_filter[THR]<-500+DEAD_NAV_RC)
		    cnt[0]++;
			else
				cnt[0]=0;
			if(cnt[0]>1/T)
			{cnt[0]=0;state_v=SG_MID_CHECK;}
			else if(mode.auto_fly_up==0&&ALT_POS_SONAR2>SONAR_HEIGHT*1.25&&fly_ready)
			{cnt[0]=0;
			if(mode.flow_hold_position!=0)
			state_v=SD_HOLD1;
			else
			state_v=SD_HOLD;	
			}
			if((mode.flow_hold_position!=0)&&(mode.auto_fly_up==0&&ALT_POS_SONAR2>SONAR_HEIGHT*1.25&&fly_ready))
			state_v=SD_HOLD1;	
			
			if(force_pass){force_pass=0;cnt[0]=0;state_v=SG_MID_CHECK;}
		break;
		case SG_MID_CHECK:
	    if(mode.auto_fly_up&&fabs(CH_filter[THR])<DEAD_NAV_RC)
		    cnt[0]++;
			else
				cnt[0]=0;
			if(cnt[0]>1.5/T)
			{cnt[0]=0;state_v=SU_UP1;}
			else if(!mode.auto_fly_up||(ALT_POS_SONAR2>SONAR_HEIGHT*1.25&&!mode.auto_fly_up)||fly_ready)
			{cnt[0]=0;state_v=SG_LOW_CHECK;}	
			
			if(force_pass){force_pass=0;cnt[0]=0;state_v=SU_UP1;}
	  break;
		case SU_UP1:
			if(cnt[0]++>5/T||ALT_POS_SONAR2>AUTO_UP_POS_Z)
				{cnt[0]=0;state_v=SD_HOLD;}	
			 if(mode.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}//restart until land	
		break;
		case SD_HOLD1:
      if(mode.auto_fly_up&&fabs(CH_filter[THR])<DEAD_NAV_RC)
		    cnt[0]++;
			else
				cnt[0]=0;
			if(cnt[0]>0.25/T)
			{cnt[0]=0;state_v=SD_HOLD;}
		  
     if(mode.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}//restart until land				
    break;
    case SD_HOLD:
      if(!mode.auto_fly_up&&fabs(CH_filter[THR])<DEAD_NAV_RC)
		    cnt[0]++;
			else
				cnt[0]=0;
			if(cnt[0]>1.5/T)
			{cnt[0]=0;state_v=SD_HIGH_FAST_DOWN;}
		 
      if(mode.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}//restart until land				
    break;				
			
		//--------------------------
    case SD_HIGH_FAST_DOWN:
			if(cnt[0]++>6/T||ALT_POS_SONAR2<AUTO_DOWN_POS_Z)
				{cnt[0]=0;state_v=SD_CIRCLE_SLOW_DOWN;}	
				
			 if(mode.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}//restart until land		
    break;
		case SD_CIRCLE_SLOW_DOWN:
			if(cnt[0]++>2/T||ALT_POS_SONAR2<SONAR_HEIGHT*1.35)
				{cnt[0]=0;state_v=SD_CHECK_G;}	
				
			 if(mode.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}//restart until land		
    break;
		case SD_CHECK_G:
			if((fabs(ALT_VEL_BMP_UKF_OLDX)<GROUND_SPEED_CHECK&&ALT_POS_SONAR2<SONAR_HEIGHT*1.35))
		    cnt[0]++;
			else
				cnt[0]=0;
			if(cnt[0]>0.5/T||fabs(acc_3d_hg.z)>1234)
			{cnt[0]=0;state_v=SD_SHUT_DOWN;}
			
			 if(mode.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}//restart until land	
    break;
		case SD_SHUT_DOWN:
		  if((!mode.auto_fly_up&&!fly_ready&&ALT_POS_SONAR2<SONAR_HEIGHT*1.25&&(CH_filter[THR]<-500+100))||!fly_ready )
		  state_v=SG_LOW_CHECK;	
    break;
		
		//------------------------------------SAFE------------------------------------------------
		case SD_SAFE://safe out
			if((!mode.auto_fly_up&&(CH_filter[THR]<-500+100)&&ALT_POS_SONAR2<SONAR_HEIGHT*1.25)||!fly_ready )
			state_v=SG_LOW_CHECK;	
		break;
		
	}
//-------------------------------------------------------------
	 if(Rc_Get_PWM.AUX1>1500)
	  state_v=SD_HOLD;
	 else
		state_v=SD_SAFE;
//----------------------------output---------------------------
	  if(state_v==SG_LOW_CHECK)
		smart.rc.POS_MODE=0;	
		else if(state_v==SG_MID_CHECK)
	  smart.rc.POS_MODE=0;	 
	  else if(state_v== SU_UP1){
		fly_ready=1;
		smart.rc.POS_MODE=SMART_MODE_SPD;
    smart.rc.RST=2;		
		smart.spd.x=smart.spd.y=0;
		smart.spd.z=AUTO_FLY_SPD_Z;
		}
		else if(state_v==SD_HOLD1||state_v==SD_HOLD){//direct fly up
    smart.pos.x=smart_in.pos.x;			
		smart.pos.y=smart_in.pos.y;		
		smart.pos.z=smart_in.pos.z;	

		smart.spd.x=smart_in.spd.x;	
		smart.spd.y=smart_in.spd.y;		
		smart.spd.z=smart_in.spd.z;		
		
		smart.rc.PITCH=smart_in.rc.PITCH;
		smart.rc.ROLL=smart_in.rc.ROLL;
		smart.rc.THROTTLE =smart_in.rc.THROTTLE;
		smart.rc.YAW=smart_in.rc.YAW;
		if(mode.flow_hold_position==2){
		smart.rc.RST=smart_in.rc.RST;		
		smart.rc.POS_MODE=smart_in.rc.POS_MODE;}
    else
    {
		smart.rc.RST=0;		
		smart.rc.POS_MODE=0;
		}	
	 }		
    else if(state_v==SD_HIGH_FAST_DOWN){
		smart.pos.x=POS_UKF_X;			
		smart.pos.y=POS_UKF_Y;				
		smart.pos.z=AUTO_DOWN_POS_Z;	
	
		smart.rc.RST=3;		
		smart.rc.POS_MODE=SMART_MODE_POS;
	  }
    else if(state_v== SD_CIRCLE_SLOW_DOWN||state_v== SD_CHECK_G)
		{
		smart.spd.x=0;			
		smart.spd.y=0;				
		smart.spd.z=-AUTO_DOWN_SPD_Z;	
	
		smart.rc.RST=3;		
		smart.rc.POS_MODE=SMART_MODE_SPD;
	  }
    else if(state_v== SD_SHUT_DOWN){
	  fly_ready =0;
		smart.rc.RST=0;		
		smart.rc.POS_MODE=0;
		}			
    else 
		{
		smart.rc.RST=0;		
		smart.rc.POS_MODE=0;
		}
	
}
