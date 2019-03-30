#include <driver_control.h>




/*爪子电机左*/
const float MOVE_CLAW_LEFT_P = 5;
const float MOVE_CLAW_LEFT_I = 0.1;
const float MOVE_CLAW_LEFT_D = 0;
const float MOVE_CLAW_LEFT_MAXOUT				= 20000;
const float MOVE_CLAW_LEFT_INTEGRATION_LIMIT	= 5000;

/*爪子电机右*/
const float MOVE_CLAW_RIGHT_P = 5;
const float MOVE_CLAW_RIGHT_I = 0.1;
const float MOVE_CLAW_RIGHT_D = 0;
const float MOVE_CLAW_RIGHT_MAXOUT				= 20000;
const float MOVE_CLAW_RIGHT_INTEGRATION_LIMIT	= 5000;


/*3510-上升电机*/
const float RISE_ARM_P = 5;
const float RISE_ARM_I = 0.1;
const float RISE_ARM_D = 0;
const float RISE_ARM_MAXOUT				= 5000;
const float RISE_ARM_INTEGRATION_LIMIT	= 2000;


pid_t g_move_claw_left_pid;
pid_t g_move_claw_right_pid;
pid_t g_rise_arm_left_pid;
pid_t g_rise_arm_right_pid;


void pid_init_all(void)
{

/*claw-left-2006*/	pid_struct_init(&g_move_claw_left_pid, POSITION_PID, SPEED_LOOP, MOVE_CLAW_LEFT_MAXOUT, MOVE_CLAW_LEFT_INTEGRATION_LIMIT,
					0, MOVE_CLAW_LEFT_P,  MOVE_CLAW_LEFT_I , MOVE_CLAW_LEFT_D);
					
/*claw-right-2006*/	pid_struct_init(&g_move_claw_right_pid, POSITION_PID, SPEED_LOOP, MOVE_CLAW_RIGHT_MAXOUT, MOVE_CLAW_RIGHT_INTEGRATION_LIMIT,
					0, MOVE_CLAW_RIGHT_P,  MOVE_CLAW_RIGHT_I , MOVE_CLAW_RIGHT_D);	
					
/*rise_arm_left_3510*/	pid_struct_init(&g_rise_arm_left_pid, POSITION_PID, SPEED_LOOP, RISE_ARM_MAXOUT, RISE_ARM_INTEGRATION_LIMIT,
					0, RISE_ARM_P,  RISE_ARM_I, RISE_ARM_D);

/*rise_arm_right_3510*/	pid_struct_init(&g_rise_arm_right_pid, POSITION_PID, SPEED_LOOP, RISE_ARM_MAXOUT, RISE_ARM_INTEGRATION_LIMIT,
					0, RISE_ARM_P,  RISE_ARM_I, RISE_ARM_D);	
	
}




/****
    *@brief 限幅
    *@param[in] object   需要限幅对象
    *@param[in] abs_max	 限幅值
    */
static void abs_limit(float *object, float abs_max)
{
    if(*object > abs_max)  *object =  abs_max;
    if(*object < -abs_max) *object = -abs_max;
}


/****
    *@brief 死区
    *@param[in] object      对象（误差）
    *@param[in] dead_lim	死区值
    */
static void dead_limit(float *object, float dead_lim)
{
	if(((*object>0)?(*object):(-*object)) < dead_lim)  *object = 0;
		else *object=*object;
}

/****
	*@brief pid参数初始化函数 内部函数
	*@param[in] pid初值
	*/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
	pid->feedback_loop = loop;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}

/****
	*@brief pid参数修改函数 内部函数
	*@param[in] KP KI KD
	*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/****
	*@brief pid结构体初始化
	*@param[in] KP KI KD
	*/

void pid_struct_init(
    pid_t* pid,
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,

    float	target,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*初始化函数指针*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset  = pid_reset;
	pid->target	  	  = target;
		
    pid->f_param_init(pid, mode,loop, maxout, intergral_limit, kp, ki, kd);//初始化PID的基本参数
	
}


/****
	*@brief pid计算函数
	*@param[in] 内环pid  外环pid
	*/
float pid_calculate(pid_t * pidInner,pid_t * pidOuter)
{
    
    if(pidInner->pid_mode == POSITION_PID) //位置式PID
    {
			
		/*****************************  角度外环  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;	 //当前角度误差
			
			//dead_limit(&(pidOuter->err[NOW]),DEADBAND);								 //死区限制
			
		pidOuter->pout  = pidOuter->p *  pidOuter->err[NOW];				 //比例输出
		pidOuter->iout += pidOuter->i  *  pidOuter->err[NOW];
		pidOuter->dout  = pidOuter->d * (pidOuter->err[NOW]-pidOuter->err[LAST]);
			
		abs_limit(&(pidOuter->iout),pidOuter->IntegralLimit);
		pidOuter->pos_out = (pidOuter->pout + pidOuter->iout + pidOuter->dout);
		abs_limit(&(pidOuter->pos_out),pidOuter->MaxOutput);
			
		pidOuter->err[LAST]=pidOuter->err[NOW];

		/*****************************  速度内环  *****************************/
		if(pidInner->feedback_loop == DOUBLE_LOOP || pidInner->feedback_loop == ANGLE_LOOP)
		{
			pidInner->err[NOW] = pidOuter->pos_out - pidInner->measure;	 //当前双环速度误差值
		}
		else if(pidInner->feedback_loop == SPEED_LOOP)
		{
			pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //当前速度误差值
		}
		pidInner->pout  = pidInner->p *  pidInner->err[NOW];			 //比例输出
		pidInner->iout += pidInner->i *  pidInner->err[NOW];		 	 //微分输出
		pidInner->dout  = pidInner->d * (pidInner->err[NOW]-pidInner->err[LAST]);//积分输出
			
		abs_limit(&(pidInner->iout),pidInner->IntegralLimit);		//积分限幅
		pidInner->pos_out = (pidInner->pout + pidInner->iout + pidInner->dout);	//位置式PID输出
		abs_limit(&(pidInner->pos_out),pidInner->MaxOutput);		//位置式PID输出限幅
			
		pidInner->err[LAST]=pidInner->err[NOW];
			
    }

		
		
    else if(pidInner->pid_mode == DELTA_PID)//增量式PID
    {
		
		/*****************************  角度外环  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;	 //当前角度误差值
			
		pidOuter->pout = pidOuter->p * (pidOuter->err[NOW] - pidOuter->err[LAST]);
		pidOuter->iout = pidOuter->i *  pidOuter->err[NOW];
		pidOuter->dout = pidOuter->d * (pidOuter->err[NOW] - 2*pidOuter->err[LAST] + pidOuter->err[LLAST]);
        
		abs_limit(&(pidOuter->iout), pidOuter->IntegralLimit);
		pidOuter->delta_u = pidOuter->pout + pidOuter->iout + pidOuter->dout;
		pidOuter->delta_out = pidOuter->last_delta_out + pidOuter->delta_u;
		abs_limit(&(pidOuter->delta_out), pidOuter->MaxOutput);
		pidOuter->last_delta_out = pidOuter->delta_out;	//更新上次增量式的
			
		pidOuter->err[LLAST]=pidOuter->err[LAST];
		pidOuter->err[LAST]=pidOuter->err[NOW];
			
			/*****************************  速度内环  *****************************/
		pidInner->err[NOW] = pidOuter->delta_out - pidInner->measure;	 //当前速度误差值
			//pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //当前速度误差值
			
		pidInner->pout = pidInner->p * (pidInner->err[NOW] - pidInner->err[LAST]);
		pidInner->iout = pidInner->i *  pidInner->err[NOW];
		pidInner->dout = pidInner->d * (pidInner->err[NOW] - 2*pidInner->err[LAST] + pidInner->err[LLAST]);
        
		abs_limit(&(pidInner->iout), pidInner->IntegralLimit);
		pidInner->delta_u   = pidInner->pout           + pidInner->iout   + pidInner->dout;
		pidInner->delta_out = pidInner->last_delta_out + pidInner->delta_u;
		abs_limit(&(pidInner->delta_out), pidInner->MaxOutput);
		pidInner->last_delta_out = pidInner->delta_out;	//更新上次增量式的值
			
		pidInner->err[LLAST] = pidInner->err[LAST];
		pidInner->err[LAST]  = pidInner->err[NOW];
			
    }
		
		
		
	else if(pidInner->pid_mode == ANALOG_PID)//模拟量PID
	{			
		/*****************************  角度外环  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;//角度误差
			
		pidOuter->integ += pidOuter->err[NOW]*pidOuter->dt;		 //模拟积分
		abs_limit(&(pidOuter->integ),pidOuter->IntegralLimit); //积分限幅
		pidOuter->deriv  = (pidOuter->err[NOW] - pidOuter->err[LAST])/pidOuter->dt;//模拟微分
			
		pidOuter->pout = pidOuter->p * pidOuter->err[NOW];
		pidOuter->iout = pidOuter->i * pidOuter->integ;
		pidOuter->dout = pidOuter->d * pidOuter->deriv;
			
		pidOuter->analog_out = pidOuter->pout+pidOuter->iout+pidOuter->dout;//模拟输出
		abs_limit(&(pidOuter->analog_out),pidOuter->MaxOutput);
			
		pidOuter->err[LAST]  = pidOuter->err[NOW];
			
			
		/*****************************  速度内环  *****************************/
		pidInner->err[NOW] = pidOuter->analog_out - pidInner->measure;//速度误差
		//pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //当前误差值
			
		pidInner->integ += pidInner->err[NOW]*pidInner->dt;		 //模拟积分
		abs_limit(&(pidInner->integ),pidInner->IntegralLimit); //积分限幅
		pidInner->deriv  = (pidInner->err[NOW] - pidInner->err[LAST])/pidInner->dt;//模拟微分
			
		pidInner->pout = pidInner->p * pidInner->err[NOW];
		pidInner->iout = pidInner->i * pidInner->integ;
		pidInner->dout = pidInner->d * pidInner->deriv;
			
		pidInner->analog_out = pidInner->pout+pidInner->iout+pidInner->dout;//模拟输出
		abs_limit(&(pidInner->analog_out),pidInner->MaxOutput);
			
		pidInner->err[LAST]  = pidInner->err[NOW];
	}
		
		
	if      (pidInner->pid_mode == POSITION_PID) 
	{
		if(pidInner->feedback_loop == SPEED_LOOP || pidInner->feedback_loop == DOUBLE_LOOP) 
			return pidInner->pos_out;
		else if(pidInner->feedback_loop == ANGLE_LOOP) 
			return pidOuter->pos_out;
	}
	else if (pidInner->pid_mode == DELTA_PID)    return pidInner->delta_out;
	else if	(pidInner->pid_mode == ANALOG_PID)   return pidInner->analog_out;
	return 0;
}



