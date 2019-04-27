#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H

#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <driver_arm.h>
#include <driver_claw.h>

#define CONTROL_TASK_CYCLE 10
#define UPDATE_DATA_CYCLE 10

/*爪子电机*/
#define MOVE_CLAW_INNER_P  8
#define MOVE_CLAW_INNER_I  4
#define MOVE_CLAW_INNER_D  3
#define MOVE_CLAW_OUTER_P  100
#define MOVE_CLAW_OUTER_I  0
#define MOVE_CLAW_OUTER_D  0
#define MOVE_CLAW_OUTER_MAXOUT				 1000
#define MOVE_CLAW_OUTER_INTEGRATION_LIMIT	 400
#define MOVE_CLAW_INNER_MAXOUT				 20000
#define MOVE_CLAW_INNER_INTEGRATION_LIMIT	 5000

/*取弹过程的时间管理*/
#define TAKE_BULLET_START 0
#define TAKE_BULLET_FIRST_OUT 150
#define TAKE_BULLET_CLAMP 50
#define TAKE_BULLET_BACK  150
#define TAKE_BULLET_FIR_IN    150
#define TAKE_BULLET_SEC_OUT   150
#define TAKE_BULLET_LOSE    80
#define TAKE_BULLET_SEC_IN  20

/*登岛过程的时间管理*/


typedef struct
{
	ArmRise_t arm_moto_left;
	ArmRise_t arm_moto_right;
	MoveClaw_t claw_moto_left;
	MoveClaw_t claw_moto_right;
	int8_t    arm_finsh_flag;  //抬升完成标志
}GetBullet_t;

void control_task(void *pvParameters);
static void control_task_init(void);
//static void move_claw_task(void);
static void bullet_pid_calculate(GetBullet_t * bullet);
static void get_buffet_task(void);
static void set_motor_current_task(void *pvParameters);
static void up_island_task(void);
static void down_island_task(void);

extern GetBullet_t g_get_bullet;

#endif 
