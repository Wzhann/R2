#include "button_push.h"
#include "main.h"
#define THE_JUDGMENT_TIMES  7

uint8_t button_last[30] = {0};//��¼��ť��һ�α�ȷ�ϵ�״̬
uint8_t button_push_tmp[30] = {0};//�����ж���������ֹ����
uint32_t button_push_cnt[30] = {0};//��¼������ť�����´���
void Record_Button_Push(void)
{
	for(int i = 0;i<= 17; i++)
		{
			//��ֻ��¼�����仯������������仯���Ͳ���¼
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
