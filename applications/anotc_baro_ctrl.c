
#include "anotc_baro_ctrl.h"
#include "filter.h"
#include "alt_fushion.h"
#include "bmp.h"
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

void fusion_prepare(float dT,float av_arr[],u16 av_num,u16 *av_cnt,float deadzone,_height_st *data,_fusion_p_st *pre_data)
{
	pre_data->dis_deadzone = my_deathzoom(data->relative_height,pre_data->dis_deadzone,deadzone);	
	Moving_Average(av_arr,av_num ,(av_cnt),(10 *pre_data->dis_deadzone ),&(pre_data->displacement)); //厘米->毫米
	
//	Moving_Average(av_arr,av_num ,(av_cnt),(10 *data->relative_height),&(pre_data->dis_deadzone)); //厘米->毫米	
//	pre_data->displacement = my_deathzoom(pre_data->dis_deadzone,pre_data->displacement,10 *deadzone);
	
	pre_data->speed = safe_div(pre_data->displacement - pre_data->displacement_old,dT,0);
	pre_data->acceleration = safe_div(pre_data->speed - pre_data->speed_old,dT,0);
	
	
	pre_data->displacement_old = pre_data->displacement;
	pre_data->speed_old = pre_data->speed;
}

void acc_fusion(float dT,_f_set_st *set,float est_acc,_fusion_p_st *pre_data,_fusion_st *fusion)
{
	fusion->fusion_acceleration.out += est_acc - fusion->est_acc_old; //估计
	anotc_filter_1(set->b1,set->g1,dT,pre_data->acceleration,&(fusion->fusion_acceleration));  //pre_data->acceleration //观测、最优
	
	fusion->fusion_speed_m.out += 1.1f *my_deathzoom(fusion->fusion_acceleration.out,0,20) *dT;
	anotc_filter_1(set->b2,set->g2,dT,pre_data->speed,&(fusion->fusion_speed_m));
	anotc_filter_1(set->b2,set->g2,dT,(-pre_data->speed + fusion->fusion_speed_m.out),&(fusion->fusion_speed_me));
	fusion->fusion_speed_me.out = LIMIT(fusion->fusion_speed_me.out,-200,200);
	fusion->fusion_speed_m.a = LIMIT(fusion->fusion_speed_m.a,-1000,1000);
	
	fusion->fusion_displacement.out += 1.05f *(fusion->fusion_speed_m.out - fusion->fusion_speed_me.out) *dT;
	anotc_filter_1(set->b3,set->g3,dT,pre_data->displacement,&(fusion->fusion_displacement));
	
	fusion->est_acc_old = est_acc;
}

//超声波融合参数

#define SONAR_AV_NUM 50
float sonar_av_arr[SONAR_AV_NUM];
u16 sonar_av_cnt;

_fusion_p_st sonar;
_fusion_st sonar_fusion;
_f_set_st sonar_f_set = {
													0.2f,
													0.5f,
													0.8f,
													
													0.2f,
													0.5f,
													0.8f	
												};

//气压计融合参数											
#define BARO_AV_NUM 100
float baro_av_arr[BARO_AV_NUM];
u16 baro_av_cnt;
_fusion_p_st baro_p;
_fusion_st baro_fusion;
_f_set_st baro_f_set = {
													0.1f,
													0.2f,
													0.3f,
													
													0.1f,
													0.1f,
													0.2f	
												};

float sonar_weight;
float wz_speed,baro_com_val;				
void baro_ctrl(float dT,_hc_value_st *height_value)
{

	MS5611_ThreadNew();
	baro.relative_height = baroAlt_fc;//baro.relative_height - 0.1f *baro_com_val;
	baro.height=MS5611_Pressure;//
  baro.h_flt=firstOrderFilter(baro.relative_height ,&firstOrderFilters[BARO_LOWPASS],dT);
			baro.h_dt = 0.02f; //气压计读取间隔时间20ms
			#if EN_ATT_CAL_FC
      baro_com_val = baro_compensate(dT,1.0f,1.0f,reference_vr_imd_down_fc[2],3500);
			#else
			baro_com_val = baro_compensate(dT,1.0f,1.0f,reference_vr_imd_down[2],3500);
			#endif
		
	    ukf_baro_task1(dT)	;
//////////////////////////////////////////				

			wz_speed = baro_fusion.fusion_speed_m.out - baro_fusion.fusion_speed_me.out;
			sonar_weight=0;
			float m_speed,f_speed;
			m_speed = (1 - sonar_weight) *baro_p.speed + sonar_weight *(sonar.speed);
			f_speed = (1 - sonar_weight) *(wz_speed) + sonar_weight *(sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out);
			
			height_value->m_acc = acc_3d_hg.z;
			height_value->m_speed = m_speed;  //(1 - sonar_weight) *hf1.ref_speed_lpf + sonar_weight *(sonar.speed);
			height_value->m_height =  baro.height;// baro_p.displacement;
			height_value->fusion_acc =  acc_body[2];//baro_fusion.fusion_acceleration.out;
			if(!mode.baro_f_use_ukfm){
			height_value->fusion_speed = my_deathzoom(LIMIT( (ALT_VEL_BMP_UKF_OLDX*1000),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,10);
			height_value->fusion_height = ALT_POS_BMP_UKF_OLDX*1000;//baro_fusion.fusion_displacement.out; 
	    }
			else
			{
			height_value->fusion_speed = my_deathzoom(LIMIT( (ALT_VEL_BMP_EKF*1000),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,10);
			height_value->fusion_height = ALT_POS_BMP_EKF*1000;//baro_fusion.fusion_displacement.out; 
			}	
			
			m100.H=(float)height_value->fusion_height/1000.;
			m100.H_Spd=(float)height_value->fusion_speed/1000.;
}




