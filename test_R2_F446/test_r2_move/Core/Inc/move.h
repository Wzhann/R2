#ifndef _MOVE_H_
#define _MOVE_H_
#include "main.h"
#include "fdRM3508.h"
#include "fdcan_bsp.h"
#include "stdio.h"
#include "string.h"
#include "robot.h"

void GET_XY_value(void);
void GET_Z_value(void);
//void Motor_Start(void);
void ball_get(void);
void change_spit(void);
void ball_spit(void);
void move_send(void);
void move_speed(void);

//void increase_spd(void);
//void decrease_spd(void);
//void stop_car(void);

#endif

