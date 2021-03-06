#include "include.h"
#include "filter.h"
#include "mymath.h"
/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x
n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

// #define WIDTH_NUM 101
// #define FIL_ITEM  10

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1)
{
	LPF_1_(gain_hz,dT,(in - f1->out),f1->a); //低通后的变化量

	f1->b = my_pow(in - f1->out);

	f1->e_nr = LIMIT(safe_div(my_pow(f1->a),((f1->b) + my_pow(f1->a)),0),0,1); //变化量的有效率
	
	LPF_1_(base_hz *f1->e_nr,dT,in,f1->out); //低通跟踪
}

 void Moving_Average1(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}


 void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out)
{
	u16 width_num;
	float last;

	width_num = len ;
	
	if( ++*fil_cnt >= width_num )	
	{
		*fil_cnt = 0; //now
	}
	
	last = moavarray[ *fil_cnt ];
	
	moavarray[ *fil_cnt ] = in;
	
	*out += ( in - ( last  ) )/(float)( width_num ) ;
	*out += 0.00001f *(in - *out);  //次要修正
	
}




s32 Moving_Median(s32 moavarray[],u16 len ,u16 *fil_p,s32 in)
{
	u16 width_num;
	u16 now_p;
	float t;
	s8 pn=0;
	u16 start_p,i;
	s32 sum = 0;

	width_num = len ;
	
	if( ++*fil_p >= width_num )	
	{
		*fil_p = 0; //now
	}
	
	now_p = *fil_p ;	
	
	moavarray[ *fil_p ] = in;
	
	if(now_p<width_num-1) //保证比较不越界
	{
		while(moavarray[now_p] > moavarray[now_p + 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p + 1];
			moavarray[now_p + 1] = t;
			pn = 1;
			now_p ++;
			if(now_p == (width_num-1))
			{
				break;
			}
		}
	}
	
	if(now_p>0)  //保证比较不越界
	{
		while(moavarray[now_p] < moavarray[now_p - 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p - 1];
			moavarray[now_p - 1] = t;
			pn = -1;
			now_p--;
			if(now_p == 0)
			{
				break;
			}
		}
	
	}
	
	if(*fil_p == 0 && pn == 1)
	{
		*fil_p = width_num - 1;
	}
	else if(*fil_p == width_num - 1 && pn == -1)
	{
		*fil_p = 0;
	}
	else
	{
		*fil_p -= pn;
	}
	
	start_p = (u16)(0.25f * width_num );
	for(i = 0; i < width_num/2;i++)
	{
		sum += moavarray[start_p + i];
	}
	return (sum/(width_num/2));
}



void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
// 	 out->x = h_tmp_x *in->x - ref->x *in->z;
// 	 out->y = ref->z *in->y - ref->y *in->z;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}
