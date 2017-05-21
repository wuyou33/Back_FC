#include "fly_mode.h"
#include "rc.h"
 ERO ero;
u8 mode_value[10];
u8 mode_state,mode_state_old;
struct _MODE mode;
void mode_check(float *ch_in,u8 *mode_value)
{
	 #if USE_RECIVER_MINE		 //使用我的手柄   未使用
		if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode = 1;//气压计
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode = 2;//超声波
			}
			else
			{
				height_ctrl_mode = 1;
			}
		}	
#else
    //定高模式判断
		if(Rc_Get_PWM.HEIGHT_MODE <1200 )
		{
				if(1)//ultra_ok == 1)
			{
				height_ctrl_mode = 2;//超声波
			}
			else
			{
				height_ctrl_mode = 1;//气压计
			}
		}
		else if(Rc_Get_PWM.HEIGHT_MODE>1400 &&Rc_Get_PWM.HEIGHT_MODE <1600 )
		{
			height_ctrl_mode = 1;//气压计
		}
		else if(Rc_Get_PWM.HEIGHT_MODE >1800 )
		{
			
				height_ctrl_mode = 0;//手动
		
		}
		
		//定点模式判断
		if(Rc_Get_PWM.POS_MODE>1800)		
		mode.flow_hold_position=2;	//智能
		else if(Rc_Get_PWM.POS_MODE<1400)
		mode.flow_hold_position=0;  //手动
    else
		mode.flow_hold_position=1;	//光流		
#endif	
		
	if(*(ch_in+AUX1) <-200)
	{
		mode_state = 0;//0;
	}
	else if(*(ch_in+AUX1) >200)
	{
		mode_state = 2;
	}
	else
	{
		mode_state = 1;
	}
	
	//=========== GPS、气压定高 未使用 ===========
	if(mode_state == 0 )
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 0;
	}
	else
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 1;
	}
	//===========   ===========
	mode_state_old = mode_state; //历史模式
}
