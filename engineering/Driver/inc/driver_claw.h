#ifndef _DRIVER_CLAW_H
#define _DRIVER_CLAW_H

#include <stm32f4xx.h>
#include <driver_control.h>

#define CLAW_CONTROL_ID     0X1FF
#define CLAW_NORMAL_SPEED   1500
#define CLAW_MIN_SPEED      0

/*爪子使用2006电机*/
typedef struct
{
	float give_current; //当前赋的电流值
	float current_round;//当前编码器的圈数
	float cuttent_ecd;  //当前编码器的值
	float last_angle_target;//上次设置的角度值
	
	pid_t moto_speed_pid;
	pid_t moto_angle_pid;
}MoveClaw_t;

void control_2006_motor(int16_t moto1,int16_t moto2);
void dial_raise_paw(void);
void dial_drop_paw(void);
#endif
