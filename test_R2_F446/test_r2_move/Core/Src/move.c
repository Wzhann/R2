#include "main.h"
#include "move.h"
#include "MY_Task.h"

extern float VX;
extern float VY;
extern float VZ;
extern int max_speed;
extern int getball_spd;
extern int spitball_spd;
extern int change[5];
uint16_t TxData[4];

void move_speed()
{
	if(button_push_cnt[9]==1)
	{
		max_speed+=200;
		button_push_cnt[9]=0;
	}
	else if(button_push_cnt[11]==1)
	{
		max_speed-=200;
		button_push_cnt[11]=0;
	}
}

void GET_XY_value()
{
	VX=(handle.lx-25) *max_speed/165.0;
	VY=(handle.ly-25) *max_speed/165.0;		
}

void GET_Z_value()
{
	VZ=(handle.rx-25) * max_speed/165.0;	
}


//void Motor_Start() 
//{
//		RM3508_Set_Speed(VX-VY+VZ,1); 
//		RM3508_Set_Speed(VX+VY+VZ,2); 
//		RM3508_Set_Speed(-VX+VY+VZ,3); 
//		RM3508_Set_Speed(-VX-VY+VZ,4); 
//}

void move_send()//发送
{
	TxData[0] = NULL;
	TxData[1] = VX;
	TxData[2] = VY;
	TxData[3] = VZ;

	FDCAN_SendData(&hfdcan2, (uint8_t *)TxData, 0x050,sizeof((uint8_t *)TxData));
}

void ball_get()//取球
{
	if(button_push_cnt[4]==1)
	{
		getball_spd+=200;
		button_push_cnt[4]=0;
	}
	else if(button_push_cnt[5]==1)
	{
		getball_spd-=200;
		button_push_cnt[5]=0;
	}
	//加减速
	if(button_push_cnt[20]%2 == 1)
	{
		RM3508_Set_Speed(getball_spd,5);
		RM3508_Set_Speed(getball_spd,6);
	}
	else if(button_push_cnt[20]%2 == 0)
	{
		RM3508_Set_Speed(0,5);
		RM3508_Set_Speed(0,6);
	}
	//给转速
}

void change_spit()//实现分球
{
	if(button_push_cnt[16]==1)
	{
		change[2]*=-1;
		button_push_cnt[16]=0;
	}
}

void ball_spit()//吐球
{
	if(button_push_cnt[8]==1)
	{
		spitball_spd+=200;
		button_push_cnt[4]=0;
	}
	else if(button_push_cnt[10]==1)
	{
		spitball_spd-=200;
		button_push_cnt[5]=0;
	}
	//加减速
	
	if(button_push_cnt[21]%2 == 1)
	{
		int i=1;
		for(i=1;i<=4;i++)
		{
		RM3508_Set_Speed(change[i]*spitball_spd,1);
		RM3508_Set_Speed(change[i]*spitball_spd,2);
		RM3508_Set_Speed(change[i]*spitball_spd,3);
		RM3508_Set_Speed(change[i]*spitball_spd,4);
		}
	}
	else if(button_push_cnt[21]%2 == 0)
	{
		RM3508_Set_Speed(0,1);
		RM3508_Set_Speed(0,2);
		RM3508_Set_Speed(0,3);
		RM3508_Set_Speed(0,4);
	}
	//给转速
}
