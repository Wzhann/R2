#ifndef __TASK_FROM_OLD_VERSIONRR_H__
#define __TASK_FROM_OLD_VERSIONRR_H__
#include "stdint.h"
#include "robot.h"
#include "gpio.h"
#include "fdcan.h"

extern uint32_t button_push_cnt[30];//记录每个按钮被按下的次数

void Record_Button_Push(void);//记录按钮被按下次数的函数

#endif
