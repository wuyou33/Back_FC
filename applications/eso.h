#ifndef _ESO_H_
#define	_ESO_H_
#include "math.h"
#include "stm32f4xx.h"
/*ADRC controll designed by Golaced from ÔÆÒÝ´´ÐÂ
*/
#define ESO_PARA_USE_REAL_TIME 1
typedef struct
{ //control para
	float KP,KD,KI;
	float b0,b01;
	float err_limit;
	float eso_dead;
	//mode
	 u8 init;
	 u8 level;
	 u8 not_use_px4;
	 u8 auto_b0;
	 u8 out_mode,use_td;
	 u8 eso_for_z;
	//adrc
	u8 n; 
	float h0;
	float z[3],e;
	float disturb,disturb_u,disturb_u_reg;
	float beta0,beta1,beta2,beta3;
	float alfa1,alfa2,alfa0,tao;	
	float h,integer;
	float v1,v2,r0,h1,r1,c,u;
	//safe
	u8 Thr_Low;
	float Thr_Weight;
}ESO;

extern ESO eso_pos[3],eso_pos_spd[3];
extern ESO eso_att_outter_c[4],eso_att_inner_c[4];

void OLDX_SMOOTH_IN_ESO(ESO *eso_in,float in);
float OLDX_AUTO_B0(ESO *eso_in,float v,float y,float u,float T,float MAX);
float OLDX_ATT_CONTRL_OUTER_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4,float kp_in,u16 thr);
float OLDX_ATT_CONTRL_INNER_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float kp_in,u16 thr);
float OLDX_POS_CONTROL_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float kp_in,u16 thr);
#endif

