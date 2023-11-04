#include "main.h"
#include "move.h"
#include "RM3508.h"
#include "communication.h"

extern float VX;
extern float VY;
extern float VZ;
extern int max_speed;


void GET_XY_value()
{
	VX=centerInfo[1];
	VY=centerInfo[2];
}

void GET_Z_value()
{
	VZ=centerInfo[3];
}
void Motor_Start() 
{
		RM3508_Set_Speed(VX-VY+VZ,1); 
		RM3508_Set_Speed(VX+VY+VZ,2); 
		RM3508_Set_Speed(-VX+VY+VZ,3); 
		RM3508_Set_Speed(-VX-VY+VZ,4); 
}

