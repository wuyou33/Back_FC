//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�����ʹ�øó�����ɷ�����ʧ�ظ���
//OLDX-AutoPilot
//SBUS�������
//��������:2017/4/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������ݴ����Ŷ� 2016-2024
//All rights reserved
//********************************************************************************


////////////////////////////////////////////////////////////////////////////////// 	 

#include "stm32f4xx.h"
extern int16_t channels[18];//SBUSͨ�����
extern uint8_t failsafe_status;//ң�����ź�״̬
//------------------------------------------------//
extern uint8_t sbus_data_i[26];
extern uint8_t sbus_data[26];
extern uint8_t sbus_passthrough;
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
//------------------------------------------------//
void oldx_sbus_rx(u8 com_data);//���˺����ŵ�SBUS�����ж�