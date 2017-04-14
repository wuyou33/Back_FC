#ifndef _CIRCLE_H_
#define	_CIRCLE_H_

#include "stm32f4xx.h"
extern float yaw_qr_off;
typedef struct
{ float exp;
	float now;
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;
	float damp;
	float err_weight;
  u8 mode;
}_pos_control;

typedef struct
{ float f_kp;
	float kp;
	float kd;
	float ki;
  float dead;
	float flt_nav_kd;
	float flt_nav;
}_pos_pid;


#define SPD_TO_MAX_AS 		2000.0f										//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define ACC_TO_MAX_AS     1000.0f
#define MAX_CTRL_POS_SPEED 	 	2000.0f									//ROL,PIT允许的最大控制角速度
#define MAX_CTRL_POS_ACC 	 	9800.0f									//ROL,PIT允许的最大控制角速度
#define NAV_POS_INT        500//mm/s  
#define NAV_SPD_INT 		0.5f *MAX_CTRL_POS_SPEED		//内环积分幅度
#define NAV_ACC_INT 		0.5f *MAX_CTRL_POS_ACC		//内环积分幅度
extern float  target_position[2];
extern float  now_position[2];
extern _pos_pid nav_pos_pid;
extern _pos_pid nav_spd_pid,nav_acc_pid;
extern _pos_control nav_pos_ctrl[2];
extern _pos_control nav_spd_ctrl[2];
void Positon_control(float T);//     光流定点 
void reset_nav_pos(u8 sel);

//---------------------
 typedef struct
{
int x,y,z;
int spdx,spdy,spdz;
int x_flp,y_flp;
u8 check;
u8 connect,lose_cnt;
int control[2];
u16 r;
float control_k_miss,control_k;
float control_yaw;
float forward;
float forward_end_dj_pwm;
u8 dj_fly_line;
float pit,rol,yaw;
u8 use_spd;
}CIRCLE;


 typedef struct
{
 int x,y;
 int x_flp,y_flp;
 u8 check;
 u8 connect,lose_cnt;
 int control[2];
 
 int pos_set[2];
 int pos_now[2];
 int angle[3];
 int Yaw_marker;
		float ero_m[2];
	float target_speed_m[2];
}MARKER;

extern CIRCLE circle,track;
extern MARKER marker;
extern float nav_circle[2],d_flow_watch[2];;
extern void circle_control(float T);
extern float Yaw_DJ,Pitch_DJ;	
#define MID_Y 152-40+14
#define MID_X 182+60-38
extern u16 Rc_Pwm_Out_mine[8];
void AUTO_LAND_FLYUP(float T);
extern float nav_land[3], nav[2];;
extern u8 state_v;
extern u8 mode_change;
#endif