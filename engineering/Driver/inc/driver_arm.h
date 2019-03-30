#ifndef _DRIVER_ARM_H
#define _DRIVER_ARM_H

#include <stm32f4xx.h>
#include <driver_control.h>

#define ARM_CONTROL_ID		0X1FF

#define ARM_NORMAL_SPEED   1000 //臂抬升的正常速度
#define ARM_MIN_SPEED		0	//臂抬升的最小速度
#define ARM_TOTAL_DISTANCE 0x12	//臂上升的总距离

typedef struct
{
	int32_t round;
	float target;
	float measure;
	int16_t give_current;
}ArmRise_t;


void set_motor_current(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4);
void test_3510_motor(void);

#endif 
