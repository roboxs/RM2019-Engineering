#ifndef _DRIVER_ISLAND_H
#define _DRIVER_ISLAND_H

#include <stm32f4xx.h>
#include <sys.h>

#define PUSH_ROD_SPEED_MIN  600 
#define ASSIST_SPEED_MIN 500


#define PUSH_ROD_MOTOR_LEFT_EN    PHout(11)//C
#define PUSH_ROD_MOTOR_IN1 		  PHout(10)//D
#define PUSH_ROD_MOTOR_IN2		  PDout(15)//E
#define PUSH_ROD_MOTOR_IN3		  PDout(14)//F
#define PUSH_ROD_MOTOR_IN4		  PDout(13)//G
#define PUSH_ROD_MOTOR_RIGHT_EN   PDout(12)//H

#define ASSIST_MOTOR_LEFT_EN 		PAout(0)	//S
#define ASSIST_MOTOR_IN1			PAout(1)	//T
#define ASSIST_MOTOR_IN2			PAout(2)	//U
#define ASSIST_MOTOR_IN3			PAout(3)	//V
#define ASSIST_MOTOR_IN4			PIout(5)	//W
#define ASSIST_MOTOR_RIGHT_EN 		PIout(6)	//X


void move_push_motor(int16_t left, int16_t right);
void move_assist_motor(int16_t left, int16_t right);


#endif

