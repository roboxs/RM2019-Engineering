#ifndef _DRIVER_CLAW_H
#define _DRIVER_CLAW_H

#include <stm32f4xx.h>
#include <driver_control.h>

#define CLAW_CONTROL_ID     0X1FF
#define CLAW_NORMAL_SPEED   1500
#define CLAW_MIN_SPEED      0

/*צ��ʹ��2006���*/
typedef struct
{
	float give_current; //��ǰ���ĵ���ֵ
	float current_round;//��ǰ��������Ȧ��
	float cuttent_ecd;  //��ǰ��������ֵ
	float last_angle_target;//�ϴ����õĽǶ�ֵ
	
	pid_t moto_speed_pid;
	pid_t moto_angle_pid;
}MoveClaw_t;

void control_2006_motor(int16_t moto1,int16_t moto2);
void dial_raise_paw(void);
void dial_drop_paw(void);
#endif
