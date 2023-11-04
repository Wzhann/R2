#ifndef __ROBOT__
#define __ROBOT__
#include "nrf.h"
#include "stdbool.h"
typedef struct
{
  //��Ӧ����ֵ
	uint8_t button[28];
	//��Ӧҡ��ֵ
	int16_t lx;
	int16_t ly;
	int16_t rx;
	int16_t ry;
}Handle_Info;

extern Handle_Info handle;
extern bool nrf_mode; // nrf����ģʽ

extern uint8_t button[28]; // ����

extern uint8_t Button_Check[28];
extern uint8_t Button_Click[28];

extern int16_t nrf_trans_cmd[7]; // �������ֱ�ȫ������
extern bool flag_readrock;


void Wheels_Run(void);
void A0_Read_Click(void);

/******************************************************************
 *@ 	name		: instruction_refresh
 *@	functio	: �ֱ����ݽ���
 *@	input		: none
 *@	output	: none
 *******************************************************************/
void instruction_refresh(void);

/******************************************************************
 *@ 	name		: read_rocker
 *@	functio	: ���ҡ������		���0~3
 *@	input		: none
 *@	output	: none
 *******************************************************************/
int16_t read_rocker(int id);

/******************************************************************
 *@ 	name		: read_keys
 *@	functio	: ��ð������ݣ����� boll button[26]��
 *@	input		: none
 *@	output	: none
 *******************************************************************/
void read_keys(void);



Handle_Info* get_remote_control_point(void);


#endif
