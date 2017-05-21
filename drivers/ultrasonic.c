#include "include.h"
#include "ultrasonic.h"
#include "usart.h"


void Ultrasonic_Init()
{
  Usart3_Init(9600);	
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
            if(MAX_error1 < Distance[i+1] - Distance[i] )
            {
                MAX_error1 =  Distance[i+1] - Distance[i];
                MAX_error_targe = i;  
            }
        }
        float UltrasonicWave_Distance1=0;
        if(MAX_error_targe+1 > (measure_num+1)/2)
        {
            for(i = 0 ; i <= MAX_error_targe ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }
            UltrasonicWave_Distance1 /= (MAX_error_targe+1);
        }
        else  
        {
             for(i = MAX_error_targe + 1 ; i < measure_num ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }
            UltrasonicWave_Distance1 /= (measure_num - MAX_error_targe -1);
        }
    return  (float)UltrasonicWave_Distance1/1000.;
}


static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float sonar_values[], unsigned n);

void sonar_bubble_sort(float sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (sonar_values[j] > sonar_values[j+1]) {
				/* swap two values */
				t = sonar_values[j];
				sonar_values[j] = sonar_values[j + 1];
				sonar_values[j + 1] = t;
			}
		}
	}
}

float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);

	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}

	/* sort and return mode */

	/* copy ring buffer */
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));

	sonar_bubble_sort(sonar_temp, sonar_count);

	/* the center element represents the mode after sorting */
	return sonar_temp[sonar_count / 2];
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
			//ultra.relative_height =sonar_filter_oldx(ultra.height);
			ultra.relative_height =insert_sonar_value_and_get_mode_value((float)ultra.height/1000.);
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

