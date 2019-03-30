#ifndef _DRIVER_CONTROL_H
#define _DRIVER_CONTROL_H

#include <stm32f4xx.h>



/****************************PID结构体*************************************/

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,//位置式
    DELTA_PID,	 //增量式
	ANALOG_PID,  //模拟
};

enum
{
	ANGLE_LOOP = 0,
	SPEED_LOOP = 1,
	DOUBLE_LOOP = 2,
};



typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
	float target;				//目标值
	float measure;				//测量值
    float err[3];				//误差
	
    
    float pout;							
    float iout;							
    float dout;							
	
	float integ;				//积分
	float deriv;				//微分
	float dt;					//△t
	
	float analog_out;			//模拟输出
    
    float pos_out;				//本次位置式输出

    float delta_u;				//本次增量值
    float delta_out;			//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;
    
	float max_err;
	float deadband;				//死区
    uint32_t pid_mode;
	uint32_t feedback_loop;			//反馈环
    uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;			//积分限幅
    
    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
                    uint32_t pid_mode,
					uint32_t feedback_loop,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改

}pid_t;

/******************************PID函数初始化**************************************/

void pid_init_all(void);

float pid_calculate(pid_t * pidInner,pid_t * pidOuter);

		
void pid_struct_init(
    pid_t* pid,
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,
    
	float   target,
    float 	kp, 
    float 	ki, 
    float 	kd);


/*******************************PID变量声明***************************************/

extern pid_t g_move_claw_left_pid;
extern pid_t g_move_claw_right_pid;
extern pid_t g_rise_arm_left_pid;
extern pid_t g_rise_arm_right_pid;


	
#endif

