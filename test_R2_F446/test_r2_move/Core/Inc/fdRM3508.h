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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "fdcan.h"
    
                  
//3508 motor data
extern uint8_t RM3508_Feedback_Buf[8][7];	 // 电机反馈值(全局变量)
extern uint8_t RM3508_Sendbuf1[8];  //CAN1发送的数据
extern uint8_t RM3508_Sendbuf2[8];  //CAN2发送的数据



/*3508电机的PID结构体*/
typedef struct
{
	float Target;
	float NowIS;

	float Err;
	float Err_last;
	float Err_sum;
	float I_Start_Err;

	float Kp;
	float Ki;
	float Kd;

	int Max;
	int Min;

	float P_Out;
	float I_Out;
	float D_Out;
	float PID_Out;

	float DeadBand;
	float IntegralLimit;
} M3508_PID;




//从CAN接收电调传回的反馈信息
void RM3508_Get_Feedback(uint32_t std_id, uint8_t *data_p);

//获取电机转矩信息
int RM3508_Get_Torque(uint8_t motor_id);
//获取电机速度信息
int RM3508_Get_Speed(uint8_t motor_id);
//获取电机位置信息
int RM3508_Get_Pos(uint8_t motor_id);
//获取电机温度信息
//uint8_t RM3508_Temperature(uint8_t id);

//电流环（在这一环向CAN发数据）
void RM3508_Set_I(int target_i, uint8_t motor_id);
//通过速度PID设置速度
void RM3508_Set_Speed(int goal_speed,int ID);
//通过位置PID设置位置（角度位置，是经过减速箱后的位置,PID中debug时存储的Target是cnt）
void RM3508_Set_Pos(float pos,int ID);
//void RM3508_Set_Ang(float angle_t_goal,int ID);

//用于CAN中断中实时更新位置信息
void RM3508_Pos_Rec(uint8_t motor_id);
//角度转线数
int RM3508_Ang2Cnt(float angle, int ID);
//线数转角度
double RM3508_Cnt2Ang(int32_t cnt, int ID);

//RM3508专用发送函数
void RM3508_SendData(FDCAN_HandleTypeDef *hfdcan);

#endif


//接收feedback的中断回调函数，根据FDCAN口对应的FIFO不同，可能需要调整内容，建议自己写
/*
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == hfdcan1.Instance)
	{
        FDCAN_RxHeaderTypeDef RxHead;
        uint8_t RxData[8] = {0};
	
	    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHead, RxData);
    
        if(RxHead.Identifier >= 0x201 && RxHead.Identifier <= 0x208)
	    {
//		    check3508[RxHead.Identifier-0x202] = 50; //检查3508在线状态，不一定用
            RM3508_Get_Feedback(RxHead.Identifier, RxData);
	    }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(hfdcan->Instance == hfdcan2.Instance)
	{
        FDCAN_RxHeaderTypeDef RxHead;
        uint8_t RxData[8] = {0};
	
	    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHead, RxData);
    
        if(RxHead.Identifier >= 0x201 && RxHead.Identifier <= 0x208)
	    {
//		    check3508[RxHead.Identifier-0x202] = 50; //检查3508在线状态，不一定用
            RM3508_Get_Feedback(RxHead.Identifier, RxData);
	    }
    }
}
*/
