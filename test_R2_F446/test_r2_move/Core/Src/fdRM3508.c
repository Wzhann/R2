/**
  ****************************(C) COPYRIGHT 2024 UESTC_LIMITI****************************
  * @file       fdRM3508.c/h
  * @brief      RM3508的FDCAN版本
  * @note       需配合 fdcan_bsp.c/h 使用  编码格式：UTF-8
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     10.11.2023       GTY             1. 整合完成，基本与普通CAN版本的3508库内容一致

  ****************************(C) COPYLEFT 2024 UESTC_LIMITI****************************
  */

#include "fdRM3508.h"
#include "main.h"
#include "fdcan_bsp.h"
#include "string.h"
#include "math.h"

//#include "INS_task.h"
//#include "move.h"      //上一届RR里面自带的，没有用

//各种计算公式
#define CLAMP(x, lower, upper)	(x >= upper ? upper : (x <= lower ? lower : x))
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define LIMIT(x, limit) (x >= limit ? limit : (x <= -limit ? -limit : x))
#define DEADBAND(x, limit) (x >= limit ? x : (x <= -limit ? x : 0))

#define RM3508_CNT_PER_ROUND (8192) //编码器线程
#define RM3508_CNT_PER_ROUND_OUT(x) (RM3508_CNT_PER_ROUND * RM3508_Reduction_Ratio[(x - 1)])
#define RM3508_ABS(x) (x >= 0 ? x : -x)

#define SEND_ID_1 0x200  //前四个电调ID对应的sendID标识符是0x200
#define SEND_ID_2 0x1ff  //后四个电调ID对应的sendID标识符是0x1ff


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
const uint8_t RM3508_Reduction_Ratio[8] = {19,19,19,19,19,19,19,19};//电机减速比数组
uint8_t RM3508_Sendbuf1[8] = {0};       //从FDCAN1发送的数据
uint8_t RM3508_Sendbuf2[8] = {0};       //从FDCAN2发送的数据
uint8_t RM3508_Feedback_Buf[8][7];	    //电机反馈值(全局变量)
int32_t RM3508_Pos[8];					//记录电机积累的位置
static int32_t	RM3508_base[8] = {0};	//用来记录电机已经转过的编码器圈数，一圈线数8192

M3508_PID M3508_Speed_Pid[8] =
{
    {.Kp = 8, .Ki = 0, .Kd = 0, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 5},	// ID = 1
//    {.Kp = 19, .Ki = 0.6f, .Kd = 5.6f, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 5},	// ID = 1
    {.Kp = 8, .Ki = 0, .Kd = 0, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 5},	// ID = 2
    {.Kp = 8, .Ki = 0, .Kd = 0, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0},	// ID = 3
    {.Kp = 8, .Ki = 0, .Kd = 0, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0},	// ID = 4
    {.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0},		// ID = 5
	{.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0},		// ID = 6
    {.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0},		// ID = 7
    {.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0},		// ID = 8
};

M3508_PID M3508_Pos_Pid[8] =
{
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2400, .Min = -2400, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 1},		// ID = 1
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2400, .Min = -2400, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 1},		// ID = 2
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2400, .Min = -2400, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 1},		// ID = 2
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2400, .Min = -2400, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 1},		// ID = 4
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 1400, .Min = -1400, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},		// ID = 5
  	{.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2200, .Min = -3000, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},		// ID = 6
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2400, .Min = -2400, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},		// ID = 7
    {.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 2400, .Min = -2400, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},		// ID = 8

};



/*********************************************************************************
 *@  name      : RM3508_Get_Feedback
 *@  function  : 获取RM3508电机的反馈并存入全局变量RM3508_Feedback_Buf[8][7];
 *@  input     : message_id,message数组指针
 *@  output    : 无
 *********************************************************************************/
void RM3508_Get_Feedback(uint32_t std_id, uint8_t *data_p)
{
	int i;
	for (i = 1; i < 9; i++)
	{
		if (std_id == 0x200 + i)
		{
			memcpy(RM3508_Feedback_Buf[i - 1], data_p, 7);
			RM3508_Pos_Rec(i);
			return;
		}
	}
}

/*********************************************************************************
  *@  name      : RM3508_Get_Torque
  *@  function  : 获取RM3508电机的实际转矩信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的转矩
*********************************************************************************/
int RM3508_Get_Torque(uint8_t motor_id)
{
	int torque = 0;
	if(RM3508_Feedback_Buf[motor_id-1][2]>>7==1)
		torque = -( 0xffff-(  (RM3508_Feedback_Buf[motor_id-1][4]<<8)+RM3508_Feedback_Buf[motor_id-1][5])  ) ;
	else 
		torque = (RM3508_Feedback_Buf[motor_id-1][4]<<8)+RM3508_Feedback_Buf[motor_id-1][5];
	return torque;
}

/*********************************************************************************
  *@  name      : RM3508_Get_Speed
  *@  function  : 获取RM3508电机的反馈的速度信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的速度
*********************************************************************************/
int RM3508_Get_Speed(uint8_t motor_id)
{
	int speed = 0;
	if(RM3508_Feedback_Buf[motor_id-1][2]>>7==1)
		speed = -( 0xffff-(  (RM3508_Feedback_Buf[motor_id-1][2]<<8)+RM3508_Feedback_Buf[motor_id-1][3])  ) ;
	else 
		speed = (RM3508_Feedback_Buf[motor_id-1][2]<<8)+RM3508_Feedback_Buf[motor_id-1][3];
	return speed;
}

/*********************************************************************************
  *@  name      : RM3508_Get_Pos
  *@  function  : 获取RM3508电机当前的位置信息
  *@  input     : 电机id号
  *@  output    : 对应id电机的位置，编码器的CNT值
*********************************************************************************/
int RM3508_Get_Pos(uint8_t motor_id)
{
	return RM3508_Pos[motor_id - 1];
}

/*********************************************************************************
 *@  name      : RM3508_Set_I
 *@  function  : RM3508电机电流设置
 *@  input     : 目标电流，电机id
 *@  output    : 无
 *********************************************************************************/
void RM3508_Set_I(int target_i, uint8_t motor_id)
{

	if (motor_id <= 4) // 前四个ID对应的sendID标识符是0x200
	{
		if( target_i<=-16384 )
			target_i=-16384;
		else if( target_i>=16384 )
			target_i=16384;
		
	}
	if( motor_id >= 1 && motor_id <= 8 )
    {
        if(motor_id <= 4) //前四个ID对应的sendID标识符是0x200
        {
            RM3508_Sendbuf1[2 * motor_id - 2] = target_i >> 8;          //电流值高8位
            RM3508_Sendbuf1[2 * motor_id - 1] = target_i & 0x00ff;      //电流值低8位

        }
        else 			  //后四个ID对应的sendID标识符是0x1ff
        {
            motor_id -= 4;            
            RM3508_Sendbuf2[2 * motor_id - 2] = target_i >> 8;          //电流值高8位
            RM3508_Sendbuf2[2 * motor_id - 1] = target_i & 0x00ff;      //电流值低8位			
        }
	}
//		if(motor_id == 0x200)//ID1-4由can1发
//		 RM3508_SendData(&hfdcan1);
//    else
//		 RM3508_SendData(&hfdcan2);
//      

}

/********************************************************************************
 *@  name      : RM3508_Set_Speed
 *@  function  : RM3508速度设置
 *@  input     : 目标速度（-15000~15000都可接受），电机id
 *@  output    : 无
 ********************************************************************************/
 void RM3508_Set_Speed(int goal_speed, int ID)
{
	uint8_t id = ID - 1;
	M3508_Speed_Pid[id].Target = goal_speed;
	M3508_Speed_Pid[id].NowIS = RM3508_Get_Speed(ID);
	M3508_Speed_Pid[id].Err_last = M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].Err = M3508_Speed_Pid[id].Target - M3508_Speed_Pid[id].NowIS;

	if (RM3508_ABS(M3508_Speed_Pid[id].Err) < M3508_Speed_Pid[id].DeadBand)
	{
		M3508_Speed_Pid[id].Err = 0;
	}

	M3508_Speed_Pid[id].Err_sum += M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].Err_sum = CLAMP(M3508_Speed_Pid[id].Err_sum, -M3508_Speed_Pid[id].IntegralLimit, M3508_Speed_Pid[id].IntegralLimit);

	M3508_Speed_Pid[id].P_Out = M3508_Speed_Pid[id].Kp * M3508_Speed_Pid[id].Err;
	M3508_Speed_Pid[id].I_Out = M3508_Speed_Pid[id].Ki * M3508_Speed_Pid[id].Err_sum;
	M3508_Speed_Pid[id].D_Out = M3508_Speed_Pid[id].Kd * (M3508_Speed_Pid[id].Err - M3508_Speed_Pid[id].Err_last);

	M3508_Speed_Pid[id].PID_Out = M3508_Speed_Pid[id].P_Out + M3508_Speed_Pid[id].I_Out + M3508_Speed_Pid[id].D_Out;
	M3508_Speed_Pid[id].PID_Out = CLAMP(M3508_Speed_Pid[id].PID_Out, M3508_Speed_Pid[id].Min, M3508_Speed_Pid[id].Max);

	RM3508_Set_I(M3508_Speed_Pid[id].PID_Out, ID);
}

/********************************************************************************
 *@  name      : RM3508_Set_Pos
 *@  function  : rm3508位置设置
 *@  input     : 目标角度（任意角度），电机id
 *@  output    : 无
 ********************************************************************************/
void RM3508_Set_Pos(float pos_goal, int ID)
{
	uint8_t id = ID - 1;

	


	M3508_Pos_Pid[id].Target = pos_goal;
	M3508_Pos_Pid[id].NowIS = RM3508_Get_Pos(ID);
	M3508_Pos_Pid[id].Err_last = M3508_Pos_Pid[id].Err;
	M3508_Pos_Pid[id].Err = DEADBAND(pos_goal - M3508_Pos_Pid[id].NowIS, M3508_Pos_Pid[id].DeadBand);

	if (fabsf(M3508_Pos_Pid[id].Err) < M3508_Pos_Pid[id].I_Start_Err)//当误差小于一定程度时开始使用KI进行调节
		M3508_Pos_Pid[id].Err_sum += M3508_Pos_Pid[id].Err;
	else//远处不使用KI进行调节
		M3508_Pos_Pid[id].Err_sum = 0;

	M3508_Pos_Pid[id].P_Out = M3508_Pos_Pid[id].Kp * M3508_Pos_Pid[id].Err;
	M3508_Pos_Pid[id].I_Out = LIMIT(M3508_Pos_Pid[id].Ki * M3508_Pos_Pid[id].Err_sum, M3508_Pos_Pid[id].IntegralLimit);
	M3508_Pos_Pid[id].D_Out = M3508_Pos_Pid[id].Kd * (M3508_Pos_Pid[id].Err - M3508_Pos_Pid[id].Err_last);

	M3508_Pos_Pid[id].PID_Out = LIMIT(M3508_Pos_Pid[id].P_Out + M3508_Pos_Pid[id].I_Out + M3508_Pos_Pid[id].D_Out, M3508_Pos_Pid[id].Max);
	
	RM3508_Set_Speed(M3508_Pos_Pid[id].PID_Out, ID);
}

/*********************************************************************************
 *@  name      : RM3508_Pos_Rec
 *@  function  : 获取RM3508电机的反馈的位置信息  //累积路程
 *@  input     : 电机id号
 *@  output    : 无
 *********************************************************************************/

void RM3508_Pos_Rec(uint8_t motor_id)
{
	int id=motor_id-1;
	int32_t	RM3508_tmp[8];
	
	static int32_t RM3508tmp_pre[8] = {0};

	RM3508_tmp[id]=(RM3508_Feedback_Buf[id][0]<<8)+RM3508_Feedback_Buf[id][1];
	if ( RM3508_tmp[id] - RM3508tmp_pre[id] > 4095 )  //转过8191到0时记录圈数
		RM3508_base[id] -= 8191;
	else if ( RM3508_tmp[id] - RM3508tmp_pre[id] < -4095 )
		RM3508_base[id] += 8191;
	
	RM3508tmp_pre[id] = RM3508_tmp[id];
	RM3508_Pos[id] = RM3508_base[id] + RM3508_tmp[id];	
	
}

/********************************************************************************
  *@  name      : RM3508_Ang2Cnt
  *@  function  : 角度转换为实际电机应该转动的位置数 //未经减速箱的转轴转一圈数值为8192
  *@  input     : 目标角度（任意角度），电机id  //id不同，减速比不同
  *@  output    : 电机位置
********************************************************************************/
int RM3508_Ang2Cnt(float angle,int ID)  
{

	int cnt;
	cnt = (int)(RM3508_CNT_PER_ROUND_OUT(ID) * angle/360);
	return cnt;
}

/********************************************************************************
  *@  name      : RM3508_Cnt2Ang
  *@  function  : 电机位置转换为角度 //未经减速箱的转轴转一圈数值为8192
  *@  input     : 电机位置，电机id  //id不同，减速比不同
  *@  output    : 电机转过的角度
********************************************************************************/
double RM3508_Cnt2Ang(int32_t cnt,int ID)
{
	double angled;
	angled = (double)((cnt * 360.0)/RM3508_CNT_PER_ROUND_OUT(ID));
	return angled;
}

/********************************************************************************
  *@  name      : RM3508_SendData
  *@  function  : 向电机发送指令
  *@  input     : 目标FDCAN口，1~4号电机由FDCAN1发，5~8号电机由FDCAN2发
  *@  output    : 无
********************************************************************************/
void RM3508_SendData(FDCAN_HandleTypeDef *hfdcan)
{
//	if(hfdcan->Instance == hfdcan1.Instance )
//	{
//		FDCAN_SendData(&hfdcan1,RM3508_Sendbuf1,SEND_ID_1,8);
//	}
	if(hfdcan->Instance == hfdcan1.Instance )
	{
		FDCAN_SendData(&hfdcan1,RM3508_Sendbuf1,SEND_ID_1,8);
	}
	else if(hfdcan->Instance == hfdcan2.Instance )
	{
		FDCAN_SendData(&hfdcan2,RM3508_Sendbuf2,SEND_ID_2,8);
	}
}


