#ifndef _TASK_CHASSIS_H
#define _TASK_CHASSIS_H
#include <stm32f4xx.h>
#include <driver_control.h>


#define CHASSIS_TASK_CYCLE 2 //底盘任务周期2ms
#define CHASSIS_MAX_SPEED 1500
#define CHASSIS_CONTROL_ID 0x200

#define CHASSIS_PID_KP 3
#define CHASSIS_PID_KI 40
#define CHASSIS_PID_KD 0
#define CHASSIS_MAX_PID_OUTER 5000
#define CHASSIS_OUTER_INTEGRATION_LIMIT 8000


typedef struct
{
	float target;
	float measure;
	int16_t give_current;
}ChassisMotor_t;


typedef struct
{
	u8 chassis_mode;				//底盘模式
	ChassisMotor_t chassis_motor[4];//底盘电机
	pid_t motor_speed_pid[4];		//底盘速度PID
	
	float vx_measure;
	float vy_measure;
	float vw_measure;
	float vx_target;
	float vy_target;
	float vw_target;
	
}ChassisMove_t;

enum
{
	CHASSIS_RC_MODE = 0,
	CHASSIS_KEYBOARD_MODE=1,
};

void chassis_task(void *pvParameters);
static void abs_limit(float *object, float abs_max);
static void chassis_init(ChassisMove_t *chassis_move);
static void chassis_set_speed(ChassisMove_t * chassis_move);
static void mecanum_calculate(float vx,float vy,float vw, short *speed);
static void chassis_update_data(ChassisMove_t *chassis_updata);
static void chassis_pid_calculate(ChassisMove_t *chassis_move);
static void control_chassis_motor(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4);


extern ChassisMove_t g_chassis_move;
#endif

