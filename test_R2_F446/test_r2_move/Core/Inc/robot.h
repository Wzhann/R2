#ifndef __ROBOT__
#define __ROBOT__
#include "nrf.h"
#include "stdbool.h"
typedef struct
{
  //对应按键值
	uint8_t button[28];
	//对应摇杆值
	int16_t lx;
	int16_t ly;
	int16_t rx;
	int16_t ry;
}Handle_Info;

extern Handle_Info handle;
extern bool nrf_mode; // nrf工作模式

extern uint8_t button[28]; // 按键

extern uint8_t Button_Check[28];
extern uint8_t Button_Click[28];

extern int16_t nrf_trans_cmd[7]; // 解析后手柄全部数据
extern bool flag_readrock;


void Wheels_Run(void);
void A0_Read_Click(void);

/******************************************************************
 *@ 	name		: instruction_refresh
 *@	functio	: 手柄数据接收
 *@	input		: none
 *@	output	: none
 *******************************************************************/
void instruction_refresh(void);

/******************************************************************
 *@ 	name		: read_rocker
 *@	functio	: 获得摇杆数据		编号0~3
 *@	input		: none
 *@	output	: none
 *******************************************************************/
int16_t read_rocker(int id);

/******************************************************************
 *@ 	name		: read_keys
 *@	functio	: 获得按键数据，放在 boll button[26]中
 *@	input		: none
 *@	output	: none
 *******************************************************************/
void read_keys(void);



Handle_Info* get_remote_control_point(void);


#endif
