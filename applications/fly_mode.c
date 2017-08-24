#include "fly_mode.h"
#include "rc.h"
#include "ak8975.h"
ERO ero;
u8 mode_value[10];
u8 mode_state,mode_state_old;
u8 height_ctrl_mode_rc;
_MODE mode_oldx;
MOUDLE module;

void mode_check(float *ch_in,u8 *mode_value)
{
	 #if USE_RECIVER_MINE		 //使用我的手柄   未使用
		if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode_rc = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode_rc = 1;//气压计
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode_rc = 2;//超声波
			}
			else
			{
				height_ctrl_mode_rc = 1;
			}
		}	
#else
    //定高模式判断
		if(Rc_Get_PWM.HEIGHT_MODE <1200 )
		{
				if(module.sonar == 1)
			{
				height_ctrl_mode_rc = 2;//超声波
			}
			else
			{
				height_ctrl_mode_rc = 1;//气压计
			}
		}
		else if(Rc_Get_PWM.HEIGHT_MODE>1400 &&Rc_Get_PWM.HEIGHT_MODE <1600 )
		{
			height_ctrl_mode_rc = 1;//气压计
		}
		else if(Rc_Get_PWM.HEIGHT_MODE >1800 )
		{
			
				height_ctrl_mode_rc = 0;//手动
		
		}
static u16 cnt_sonar_mask;		
//auto switch for height feedback		
	if(height_ctrl_mode_rc==2&&NS==2&&0 ){
    if(ALT_POS_SONAR2<2&&cnt_sonar_mask>2/0.05&&module.sonar ==1){
			height_ctrl_mode=2;
			cnt_sonar_mask=65530;
		}
		else{
			height_ctrl_mode=1;
		}
		
		if(ALT_POS_SONAR2<1.5)
			cnt_sonar_mask++;
		else
			cnt_sonar_mask=0;
	}
	else
	height_ctrl_mode=height_ctrl_mode_rc;
		
		//定点模式判断
		if(Rc_Get_PWM.POS_MODE>1800)		
		mode_oldx.flow_hold_position=2;	//智能
		else if(Rc_Get_PWM.POS_MODE<1400)
		mode_oldx.flow_hold_position=0;  //手动
    else 
		mode_oldx.flow_hold_position=1;	//速度
   
		static u8 state_cal_en,mode_reg;
		static u16 cnt1,cnt_time;
		switch(state_cal_en)
		{
			case 0:
				if(mode_oldx.flow_hold_position!=mode_reg)
				{cnt1=0;state_cal_en=1;cnt_time=0;}
				break;
		  case 1:
				if(mode_oldx.flow_hold_position!=mode_reg)
				{cnt1=0;state_cal_en=1;cnt_time++;cnt1=0;}
				if(cnt1++>2.22/0.05)
				{cnt1=0;state_cal_en=0;}
				else if(cnt_time>8*2)
				{cnt1=0;state_cal_en=0;cnt_time=0;ak8975_fc.Mag_CALIBRATED=1;}	
			break;
		}
		mode_reg=mode_oldx.flow_hold_position;
    #if  USE_M100_IMU
		mode_oldx.flow_hold_position=mode_oldx.flow_hold_position*m100.m100_connect;	
    #endif		
#endif	

}
