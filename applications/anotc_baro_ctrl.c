#include "rc.h"
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

_fusion_p_st sonar;
_fusion_st sonar_fusion;
_fusion_p_st baro_p;
_fusion_st baro_fusion;


												
static float bmp_values[3] = { 0.0f };
static unsigned insert_index = 0;
static void bmp_bubble_sort(float bmp_values[], unsigned n);
void bmp_bubble_sort(float bmp_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (bmp_values[j] > bmp_values[j+1]) {
				t = bmp_values[j];
				bmp_values[j] = bmp_values[j + 1];
				bmp_values[j + 1] = t;
			}
		}
	}
}

float insert_bmp_value_and_get_mode_value(float insert)
{
	const unsigned bmp_count = sizeof(bmp_values) / sizeof(bmp_values[0]);

	bmp_values[insert_index] = insert;
	insert_index++;
	if (insert_index == bmp_count) {
		insert_index = 0;
	}
	float bmp_temp[bmp_count];
	memcpy(bmp_temp, bmp_values, sizeof(bmp_values));
	bmp_bubble_sort(bmp_temp, bmp_count);
	return bmp_temp[bmp_count / 2];
}


float sonar_weight;
float k_w_bmp=1;
float bmp_w;
float wz_speed,baro_com_val;				
void baro_ctrl(float dT,_hc_value_st *height_value)
{ 
      //baro.h_flt=firstOrderFilter((baro.relative_height) ,&firstOrderFilters[BARO_LOWPASS],dT);
			//baro.h_flt=baro.relative_height;
	    baro.h_dt = dT; 
			#if EN_ATT_CAL_FC
      baro_com_val = baro_compensate(dT,1.0f,1.0f,reference_vr_imd_down_fc[2],3500);
			#else
			baro_com_val = baro_compensate(dT,1.0f,1.0f,reference_vr_imd_down[2],3500);
			#endif
		
	    ukf_baro_task1(dT)	;//高度融合
//////////////////////////////////////////				

			wz_speed = baro_fusion.fusion_speed_m.out - baro_fusion.fusion_speed_me.out;
			sonar_weight=0;
			float m_speed,f_speed;
			m_speed = (1 - sonar_weight) *baro_p.speed + sonar_weight *(sonar.speed);
			f_speed = (1 - sonar_weight) *(wz_speed) + sonar_weight *(sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out);
			
			height_value->m_acc = acc_3d_hg.z;
			height_value->m_speed = m_speed;  
			height_value->m_height =  baro.height;
			height_value->fusion_acc =  acc_body[2];
			
			if(!mode.baro_f_use_ukfm){
		  float bmp_wt = LIMIT(LIMIT(fabs(Pit_fc)/30.,0,1)*LIMIT(fabs(Rol_fc)/30.,0,1),0,1);   
      bmp_w+= ( 1 / ( 1 + 1 / ( 3.14f *dT*0.5 ) ) )*( (bmp_wt - bmp_w));				
      if(height_ctrl_mode==2&&NS==2)		
			bmp_w=0;
      else{
//			if(Rc_Get_PWM.RST>1500)	
//			bmp_w=1;	
//			else
			bmp_w=0;	
		  }
			height_value->fusion_speed = my_deathzoom(LIMIT( ((ALT_VEL_BMP_UKF_OLDX*(1-bmp_w)+(bmp_w)*ALT_VEL_BMP_EKF)*1000),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,0);
			height_value->fusion_height =( ALT_POS_BMP_UKF_OLDX*(1-bmp_w)+(bmp_w)*ALT_POS_BMP_EKF)*1000;
	    }
			else
			{
			height_value->fusion_speed = my_deathzoom(LIMIT( (ALT_VEL_BMP_EKF*1000),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,0);
			height_value->fusion_height = ALT_POS_BMP_EKF*1000;
			}	
			
			m100.H=(float)height_value->fusion_height/1000.;
			m100.H_Spd=(float)height_value->fusion_speed/1000.;
}




