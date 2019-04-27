#include <driver_control.h>


/*3510-�������*/
const float RISE_ARM_P = 5;
const float RISE_ARM_I = 0.1;
const float RISE_ARM_D = 0;
const float RISE_ARM_MAXOUT				= 5000;
const float RISE_ARM_INTEGRATION_LIMIT	= 2000;


pid_t g_move_claw_left_pid;
pid_t g_move_claw_right_pid;
pid_t g_rise_arm_left_pid;
pid_t g_rise_arm_right_pid;
pid_t g_move_claw_left_speed,g_move_claw_right_angle;


void pid_init_all(void)
{	
/*rise_arm_left_3510*/	pid_struct_init(&g_rise_arm_left_pid, POSITION_PID, SPEED_LOOP, RISE_ARM_MAXOUT, RISE_ARM_INTEGRATION_LIMIT,
					RISE_ARM_P,  RISE_ARM_I, RISE_ARM_D);

/*rise_arm_right_3510*/	pid_struct_init(&g_rise_arm_right_pid, POSITION_PID, SPEED_LOOP, RISE_ARM_MAXOUT, RISE_ARM_INTEGRATION_LIMIT,
					RISE_ARM_P,  RISE_ARM_I, RISE_ARM_D);	
	
}




/****
    *@brief �޷�
    *@param[in] object   ��Ҫ�޷�����
    *@param[in] abs_max	 �޷�ֵ
    */
static void abs_limit(float *object, float abs_max)
{
    if(*object > abs_max)  *object =  abs_max;
    if(*object < -abs_max) *object = -abs_max;
}


/****
    *@brief ����
    *@param[in] object      ������
    *@param[in] dead_lim	����ֵ
    */
static void dead_limit(float *object, float dead_lim)
{
	if(((*object>0)?(*object):(-*object)) < dead_lim)  *object = 0;
		else *object=*object;
}

/****
	*@brief pid������ʼ������ �ڲ�����
	*@param[in] pid��ֵ
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
	*@brief pid�����޸ĺ��� �ڲ�����
	*@param[in] KP KI KD
	*/
void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/****
	*@brief pid�ṹ���ʼ��
	*@param[in] KP KI KD
	*/

void pid_struct_init(
    pid_t* pid,
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,

    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*��ʼ������ָ��*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset  = pid_reset;
		
    pid->f_param_init(pid, mode,loop, maxout, intergral_limit, kp, ki, kd);//��ʼ��PID�Ļ�������
	
}


/****
	*@brief pid���㺯��
	*@param[in] �ڻ�pid  �⻷pid
	*/
float pid_calculate(pid_t * pidInner,pid_t * pidOuter)
{
    
    if(pidInner->pid_mode == POSITION_PID) //λ��ʽPID
    {
			
		/*****************************  �Ƕ��⻷  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;	 //��ǰ�Ƕ����
			
			//dead_limit(&(pidOuter->err[NOW]),DEADBAND);								 //��������
			
		pidOuter->pout  = pidOuter->p *  pidOuter->err[NOW];				 //�������
		pidOuter->iout += pidOuter->i  *  pidOuter->err[NOW];
		pidOuter->dout  = pidOuter->d * (pidOuter->err[NOW]-pidOuter->err[LAST]);
			
		abs_limit(&(pidOuter->iout),pidOuter->IntegralLimit);
		pidOuter->pos_out = (pidOuter->pout + pidOuter->iout + pidOuter->dout);
		abs_limit(&(pidOuter->pos_out),pidOuter->MaxOutput);
			
		pidOuter->err[LAST]=pidOuter->err[NOW];

		/*****************************  �ٶ��ڻ�  *****************************/
		if(pidInner->feedback_loop == DOUBLE_LOOP || pidInner->feedback_loop == ANGLE_LOOP)
		{
			pidInner->err[NOW] = pidOuter->pos_out - pidInner->measure;	 //��ǰ˫���ٶ����ֵ
		}
		else if(pidInner->feedback_loop == SPEED_LOOP)
		{
			pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //��ǰ�ٶ����ֵ
		}
		pidInner->pout  = pidInner->p *  pidInner->err[NOW];			 //�������
		pidInner->iout += pidInner->i *  pidInner->err[NOW];		 	 //΢�����
		pidInner->dout  = pidInner->d * (pidInner->err[NOW]-pidInner->err[LAST]);//�������
			
		abs_limit(&(pidInner->iout),pidInner->IntegralLimit);		//�����޷�
		pidInner->pos_out = (pidInner->pout + pidInner->iout + pidInner->dout);	//λ��ʽPID���
		abs_limit(&(pidInner->pos_out),pidInner->MaxOutput);		//λ��ʽPID����޷�
			
		pidInner->err[LAST]=pidInner->err[NOW];
			
    }

		
		
    else if(pidInner->pid_mode == DELTA_PID)//����ʽPID
    {
		
		/*****************************  �Ƕ��⻷  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;	 //��ǰ�Ƕ����ֵ
			
		pidOuter->pout = pidOuter->p * (pidOuter->err[NOW] - pidOuter->err[LAST]);
		pidOuter->iout = pidOuter->i *  pidOuter->err[NOW];
		pidOuter->dout = pidOuter->d * (pidOuter->err[NOW] - 2*pidOuter->err[LAST] + pidOuter->err[LLAST]);
        
		abs_limit(&(pidOuter->iout), pidOuter->IntegralLimit);
		pidOuter->delta_u = pidOuter->pout + pidOuter->iout + pidOuter->dout;
		pidOuter->delta_out = pidOuter->last_delta_out + pidOuter->delta_u;
		abs_limit(&(pidOuter->delta_out), pidOuter->MaxOutput);
		pidOuter->last_delta_out = pidOuter->delta_out;	//�����ϴ�����ʽ��
			
		pidOuter->err[LLAST]=pidOuter->err[LAST];
		pidOuter->err[LAST]=pidOuter->err[NOW];
			
			/*****************************  �ٶ��ڻ�  *****************************/
		pidInner->err[NOW] = pidOuter->delta_out - pidInner->measure;	 //��ǰ�ٶ����ֵ
			//pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //��ǰ�ٶ����ֵ
			
		pidInner->pout = pidInner->p * (pidInner->err[NOW] - pidInner->err[LAST]);
		pidInner->iout = pidInner->i *  pidInner->err[NOW];
		pidInner->dout = pidInner->d * (pidInner->err[NOW] - 2*pidInner->err[LAST] + pidInner->err[LLAST]);
        
		abs_limit(&(pidInner->iout), pidInner->IntegralLimit);
		pidInner->delta_u   = pidInner->pout           + pidInner->iout   + pidInner->dout;
		pidInner->delta_out = pidInner->last_delta_out + pidInner->delta_u;
		abs_limit(&(pidInner->delta_out), pidInner->MaxOutput);
		pidInner->last_delta_out = pidInner->delta_out;	//�����ϴ�����ʽ��ֵ
			
		pidInner->err[LLAST] = pidInner->err[LAST];
		pidInner->err[LAST]  = pidInner->err[NOW];
			
    }
		
		
		
	else if(pidInner->pid_mode == ANALOG_PID)//ģ����PID
	{			
		/*****************************  �Ƕ��⻷  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;//�Ƕ����
			
		pidOuter->integ += pidOuter->err[NOW]*pidOuter->dt;		 //ģ�����
		abs_limit(&(pidOuter->integ),pidOuter->IntegralLimit); //�����޷�
		pidOuter->deriv  = (pidOuter->err[NOW] - pidOuter->err[LAST])/pidOuter->dt;//ģ��΢��
			
		pidOuter->pout = pidOuter->p * pidOuter->err[NOW];
		pidOuter->iout = pidOuter->i * pidOuter->integ;
		pidOuter->dout = pidOuter->d * pidOuter->deriv;
			
		pidOuter->analog_out = pidOuter->pout+pidOuter->iout+pidOuter->dout;//ģ�����
		abs_limit(&(pidOuter->analog_out),pidOuter->MaxOutput);
			
		pidOuter->err[LAST]  = pidOuter->err[NOW];
			
			
		/*****************************  �ٶ��ڻ�  *****************************/
		pidInner->err[NOW] = pidOuter->analog_out - pidInner->measure;//�ٶ����
		//pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //��ǰ���ֵ
			
		pidInner->integ += pidInner->err[NOW]*pidInner->dt;		 //ģ�����
		abs_limit(&(pidInner->integ),pidInner->IntegralLimit); //�����޷�
		pidInner->deriv  = (pidInner->err[NOW] - pidInner->err[LAST])/pidInner->dt;//ģ��΢��
			
		pidInner->pout = pidInner->p * pidInner->err[NOW];
		pidInner->iout = pidInner->i * pidInner->integ;
		pidInner->dout = pidInner->d * pidInner->deriv;
			
		pidInner->analog_out = pidInner->pout+pidInner->iout+pidInner->dout;//ģ�����
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



