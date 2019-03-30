#include <driver_island.h>

/****
	*@note :最小限制幅度
	*/
static void min_limit( int16_t *object, int16_t limit)
{
	if((*object) < limit ) *object = limit;
}

/****
	*@note :直流推杆电机速度函数
	*@param[in] : 左电机速度 右电机速度
	*/
void move_push_motor(int16_t left, int16_t right)
{
	if((left>0)&&(right>0))
	{
		PUSH_ROD_MOTOR_IN1 =1;
		PUSH_ROD_MOTOR_IN2 =0;
		PUSH_ROD_MOTOR_IN3 =0;
		PUSH_ROD_MOTOR_IN4 =1;
		left=left;
		right=right;
	}
	else if((left>0)&&(right<0))
	{
		PUSH_ROD_MOTOR_IN1 =0;
		PUSH_ROD_MOTOR_IN2 =1;
		PUSH_ROD_MOTOR_IN3 =1;
		PUSH_ROD_MOTOR_IN4 =0;
		left=left;
		right=-right;
	}
	else if( (left<0) && (right>0) )
	{
		PUSH_ROD_MOTOR_IN1 =1;
		PUSH_ROD_MOTOR_IN2 =0;
		PUSH_ROD_MOTOR_IN3 =1;
		PUSH_ROD_MOTOR_IN4 =0;
		left=-left;
		right = right;
	}
	else if( (left<0) && (right<0) )
	{
		PUSH_ROD_MOTOR_IN1 =0;
		PUSH_ROD_MOTOR_IN2 =1;
		PUSH_ROD_MOTOR_IN3 =1;
		PUSH_ROD_MOTOR_IN4 =0;
		left =-left;
		right= -right;
	}
	else if( (left==0) && (right==0) )
	{
		TIM_SetCompare2(TIM5,(uint32_t)left);
		TIM_SetCompare1(TIM4,(uint32_t)right);
		return ;
	}
	//速度最小限幅
	min_limit(&left,PUSH_ROD_SPEED_MIN);
	min_limit(&right,PUSH_ROD_SPEED_MIN);
	TIM_SetCompare2(TIM5,(uint32_t)left);
	TIM_SetCompare1(TIM4,(uint32_t)right);
}

/****
	*@note :辅助轮电机速度函数
	*@param[in] : 左电机速度 右电机速度
	*/
void move_assist_motor(int16_t left, int16_t right)
{
		if((left>0)&&(right>0))
	{
		ASSIST_MOTOR_IN1 =1;
		ASSIST_MOTOR_IN2 =0;
		ASSIST_MOTOR_IN3 =1;
		ASSIST_MOTOR_IN4 =0;
		left=left;
		right=right;
	}
	else if((left>0)&&(right<0))
	{
		ASSIST_MOTOR_IN1 =1;
		ASSIST_MOTOR_IN2 =0;
		ASSIST_MOTOR_IN3 =1;
		ASSIST_MOTOR_IN4 =0;
		left=left;
		right=-right;
	}
	else if( (left<0) && (right>0) )
	{
		ASSIST_MOTOR_IN1 =1;
		ASSIST_MOTOR_IN2 =0;
		ASSIST_MOTOR_IN3 =1;
		ASSIST_MOTOR_IN4 =0;
		left=-left;
		right = right;
	}
	else if( (left<0) && (right<0) )
	{
		ASSIST_MOTOR_IN1 =0;
		ASSIST_MOTOR_IN2 =1;
		ASSIST_MOTOR_IN3 =0;
		ASSIST_MOTOR_IN4 =1;
		left =-left;
		right= -right;
	}
	else if( (left==0) && (right==0) )
	{
		TIM_SetCompare1(TIM2,left);
		TIM_SetCompare2(TIM8,right);
		return ;
	}
	//速度最小限幅
	min_limit(&left,ASSIST_SPEED_MIN);
	min_limit(&right,ASSIST_SPEED_MIN);
	TIM_SetCompare1(TIM2,left);
	TIM_SetCompare2(TIM8,right);
}





