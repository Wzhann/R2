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

        /*����*/
        instruction_refresh( );
        /*����*/
        read_keys();
        
		/*��¼��ť�����µĴ���*/
        Record_Button_Push();

}
