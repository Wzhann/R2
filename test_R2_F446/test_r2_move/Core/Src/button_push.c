#include "button_push.h"
#include "main.h"
#define THE_JUDGMENT_TIMES  7

uint8_t button_last[30] = {0};//记录按钮上一次被确认的状态
uint8_t button_push_tmp[30] = {0};//计数判定次数，防止误判
uint32_t button_push_cnt[30] = {0};//记录各个按钮被按下次数
void Record_Button_Push(void)
{
	for(int i = 0;i<= 17; i++)
		{
			//即只记录按键变化，如果按键不变化，就不记录
			if(button_last[i] == 0 && button[i] == 1) {
				button_push_tmp[i] ++;
				if(button_push_tmp[i] > THE_JUDGMENT_TIMES)
					{
						button_push_tmp[i] = 0;
						button_last[i] = 1;
						button_push_cnt[i]++;
					}
			}
			if(button_last[i] == 1 && button[i] == 0) {
				button_push_tmp[i] ++ ;
				if(button_push_tmp[i] > THE_JUDGMENT_TIMES)
					{
					  button_push_tmp[i] = 0;
					  button_last[i] = 0;
					}
			}
		}
}
