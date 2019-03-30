#ifndef _DRIVER_CLAW_H
#define _DRIVER_CLAW_H

#include <stm32f4xx.h>

#define CLAW_CONTROL_ID     0X1FF
#define CLAW_NORMAL_SPEED   1500
#define CLAW_MIN_SPEED      0

typedef struct
{
	int32_t round;
	float target;
	float measure;
	int16_t give_current;
}MoveClaw_t;

void control_2006_motor(int16_t moto1,int16_t moto2);
void dial_raise_paw(void);
void dial_drop_paw(void);
#endif
