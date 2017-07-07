#ifndef __FLY_MODE_H
#define __FLY_MODE_H

#include "stm32f4xx.h"
#include "include.h"
#include "parameter.h"


typedef struct 
{
u8 nav_control_use_acc_loop;
u8 thr_fix;
u8 en_pid_out_pit;
u8 en_pid_out_rol;
u8 en_pid_out_yaw;
u8 en_pid_fuzzy_p;
u8 en_pid_sb_set;
u8 trig_flow_spd;
u8 trig_h_spd;
u8 en_fuzzy_angle_pid;
u8 en_sensor_equal_flp;
u8 pit_rol_pid_out_flp;	
u8 en_pid_yaw_angle_control;	
u8 en_pid_yaw_control_by_pit_rol;	
u8 thr_add_pos;
u8 spid;
u8 mpu6050_bw_42;
u8 en_imu_q_update_mine;	
u8 en_moto_smooth;
u8 pid_mode;
u8 no_head;
u8 slow_shut1;
u8 sonar_avoid;	
u8 yaw_sel;
u8 sb_smooth;
u8 use_px4_err;
u8 flow_hold_position;
u8 dji_sonar_high;
u8 auto_fly_up,auto_land;
u8 en_circle_nav,circle_miss_fly,en_track_forward,en_circle_locate;
u8 flow_hold_position_use_global;
u8 flow_hold_position_high_fix;
u8 height_safe;
u8 baro_lock;
u8 baro_f_use_ukfm;
u8 flow_f_use_ukfm;
u8 use_ano_bmp_spd;
u8 tune_ctrl_angle_offset;
u8 imu_use_mid_down;
u8 hunman_pid;
u8 yaw_imu_control;	
u8 cal_sel;
u8 flow_sel;
u8 height_in_speed;
u8 height_upload;
u8 en_h_mode_switch;
u8 en_dj_cal;
u8 en_sd_save;
u8 en_break;
u8 use_dji;
u8 en_marker;
u8 rc_control_flow_spd;
u8 rc_control_flow_pos;
u8 rc_control_flow_pos_sel;
u8 dj_by_hand;
u8 en_dj_control;
u8 dj_yaw_follow;
u8 dji_mode;
u8 en_visual_control;
u8 hold_use_flow;
u8 en_sonar_avoid;
u8 thr_fix_test;
u8 en_imu_ekf;
u8 att_pid_tune;
u8 high_pid_tune;
u8 dj_lock;
u8 en_eso;
u8 en_eso_h_out;
u8 en_eso_h_in;
u8 yaw_use_eso;
u8 flow_d_acc;
u8 en_hinf_height_spd;
u8 en_circle_control;
u8 save_video;
u8 en_h_inf;
u8 test1;
u8 test2;
u8 test3;
u8 test4;	
u8 mode_fly;
u8 h_is_fix;
u8 att_ident1;
//flow
u8 en_flow_gro_fix;
u8 flow_size;u8 show_qr_origin;
}_MODE;

extern _MODE mode_oldx;

enum
{
	BARO=0,
	GPS,
	BACK_HOME,
	//UTRASONIC,

};
typedef struct 
{
 u8 ero_att,ero_hight,ero_rst_h,ero_rst_att;
 u8 baro_ekf,baro_ekf_cnt;
	
}ERO;
extern ERO ero;
extern u8 mode_value[],mode_state;


void mode_check(float *ch_in,u8 *mode_value);
#endif
