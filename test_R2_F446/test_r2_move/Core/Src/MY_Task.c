#include "main.h"
#include "MY_Task.h"
void Program_Init(void)
{
	FDCAN1_Init(&hfdcan1);
	FDCAN2_Init(&hfdcan2);
	FDCAN2_Init(&hfdcan3);
	nrf_init();
}
void NRF_Refresh_Task(void)
{

        /*接收*/
        instruction_refresh( );
        /*解码*/
        read_keys();
        
		/*记录按钮被按下的次数*/
        Record_Button_Push();

}
