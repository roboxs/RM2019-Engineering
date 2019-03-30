#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H

#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <driver_arm.h>
#include <driver_claw.h>

#define CONTROL_TASK_CYCLE 10
#define UPDATE_DATA_CYCLE 10


typedef struct
{
	ArmRise_t arm_moto_left;
	ArmRise_t arm_moto_right;
	MoveClaw_t claw_moto_left;
	MoveClaw_t claw_moto_right;
	int8_t    arm_finsh_flag;  //抬升完成标志
}GetBullet_t;

void control_task(void *pvParameters);
static void move_claw_task(void);
static void bullet_pid_calculate(GetBullet_t * bullet);
static void get_buffet_task(void);
static void set_motor_current_task(void *pvParameters);
static void up_island_task(void);
static void down_island_task(void);

extern GetBullet_t g_get_bullet;

#endif 
