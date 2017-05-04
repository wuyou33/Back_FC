#include "include.h"
#include "ultrasonic.h"
#include "usart.h"


void Ultrasonic_Init()
{
  Usart3_Init(9600);			//串口5初始化，函数参数为波特率
}

s8 ultra_start_f;

void Ultra_Duty()
{
	u8 temp[3];

	ultra.h_dt = 0.05f; //50ms一次
	#if defined(USE_KS103)

	#elif defined(USE_US100)
	#if SONAR_USE_FC1
	  temp[0]=0x55;
	  USART_SendData(UART5, 0x55); 
	#else
		Uart3_Send(0x55);
	#endif
	#endif
///////////////////////////////////////////////
		ultra_start_f = 1;

		if(ultra.measure_ot_cnt<200) //200ms
		{
			ultra.measure_ot_cnt += ultra.h_dt *1000;
		}
		else
		{
			ultra.measure_ok = 0;//超时，复位
		}
}

u16 ultra_distance_old;

_height_st ultra;

u8 measure_num=8;
u16 Distance[50]  = {0};
float sonar_filter_oldx(float in) 
{
    u8 IS_success =1;
    u16 Distance1 = 0;
    u16 MAX_error1 = 0;
    u8 MAX_error_targe = 0;
    u8 count = 0;
    u8 i =0;
    u8 j =0;
    u8 num =0; 
    Distance[measure_num-1]=in;
    for(i=0;i<measure_num-1;i++)
		{
		 Distance[i]=Distance[i+1]; 
		}
        for(i = 0 ; i < measure_num-1 ; i++)
        {

            for(j = 0 ; j < measure_num-1-i; j++)       
            {
                if(Distance[j] > Distance[j+1] )
                {
                    Distance1 = Distance[j];
                    Distance[j] =  Distance[j+1];
                    Distance[j+1] = Distance1; 
                }
            }

        }
        MAX_error1 = Distance[1] - Distance[0];
        for(i = 1 ; i < measure_num-1 ; i++)
        {
            if(MAX_error1 < Distance[i+1] - Distance[i] )//Èç£º1 2 3 4 5    8 9 10    MAX_error_targe=4;
            {
                MAX_error1 =  Distance[i+1] - Distance[i];//×î´ó²î¾à
                MAX_error_targe = i;  //¼ÇÏÂ×î´ó²î¾àÖµµÄÎ»ÖÃ£¨Õâ×éÊýÖÐµÄÎ»ÖÃ£©
            }
        }
        float UltrasonicWave_Distance1=0;
        if(MAX_error_targe+1 > (measure_num+1)/2) //Ç°²¿·ÖÓÐÐ§  1 2 3 4 5    8 9 10  (Èç¹ûÎ»ÓÚÖÐ¼ä£¬ºó°ë²¿ÓÅÏÈ)
        {
            for(i = 0 ; i <= MAX_error_targe ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }
            UltrasonicWave_Distance1 /= (MAX_error_targe+1);//È¡Æ½¾ù
        }
        else  //ºó²¿·ÖÓÐÐ§  1 2 3   7 8 9 10
        {
             for(i = MAX_error_targe + 1 ; i < measure_num ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }
            UltrasonicWave_Distance1 /= (measure_num - MAX_error_targe -1);//È¡Æ½¾ù
        }
    return  (float)UltrasonicWave_Distance1/1000.; //×ª»¯ÎªÃ×Îªµ¥Î»µÄ¸¡µãÊý
}

void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		ultra.height =  ((ultra_tmp<<8) + com_data);
		
		if(ultra.height < 5000) // 5米范围内认为有效，跳变值约10米.
		{
			ultra.relative_height =sonar_filter_oldx(ultra.height);
			ultra.measure_ok = 1;		
		}
		else
		{
			ultra.measure_ok = 2; //数据超范围
		}
		ultra_start_f = 0;
	}
	ultra.measure_ot_cnt = 0; //清除超时计数（喂狗）
	ultra.h_delta = ultra.relative_height - ultra_distance_old;
	ultra_distance_old = ultra.relative_height;
}

