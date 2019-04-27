#ifndef _DRIVER_ARM_H
#define _DRIVER_ARM_H

#include <stm32f4xx.h>
#include <driver_control.h>

#define ARM_CONTROL_ID		0X1FF

#define ARM_NORMAL_SPEED   1000 //��̧���������ٶ�
#define ARM_MIN_SPEED		0	//��̧������С�ٶ�
#define ARM_TOTAL_DISTANCE 0x1c	//���������ܾ���

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
