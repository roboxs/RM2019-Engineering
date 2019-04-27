#include <FreeRTOS.h>
#include <task.h>
#include <task_chassis.h>
#include <driver_dbus.h>
#include <task_imu.h>
#include <task_led.h>
#include <driver_control.h>

ChassisMove_t g_chassis_move;

float chassis_kp=10,chassis_ki=1,chassis_kd=7;
void chassis_task(void *pvParameters)
{
	unsigned char i;
	//���̳�ʼ��
	chassis_init(&g_chassis_move);
	while(1)
	{
		//�������ݸ���
		chassis_update_data(&g_chassis_move);
		//�����ٶ�����
		chassis_set_speed(&g_chassis_move);
		//����PID����
		chassis_pid_calculate(&g_chassis_move);
		
		control_chassis_motor(g_chassis_move.chassis_motor[0].give_current,g_chassis_move.chassis_motor[1].give_current,
							  g_chassis_move.chassis_motor[2].give_current,g_chassis_move.chassis_motor[3].give_current);
		
		for(i=0; i<4; i++)
		{
			pid_reset(&(g_chassis_move.motor_speed_pid[i]),chassis_kp,chassis_ki,chassis_kd);
		}
		
		vTaskDelay(CHASSIS_TASK_CYCLE);
	}
}

 /****
    *@brief �޷���������
    *@param[in] object   ��Ҫ�޷�����
    *@param[in] abs_max	 �޷�ֵ
    */
static void abs_limit(float *object, float abs_max)
{
    if(*object > abs_max)  *object =  abs_max;
    if(*object < -abs_max) *object = -abs_max;
}

static void chassis_set_speed(ChassisMove_t * chassis_move)
{
	//�����ң��ģʽ
	if(chassis_move->chassis_mode == CHASSIS_RC_MODE)
	{
		chassis_move->vx_target=RC_Ctl.rc.ch3 * 2.5;
		chassis_move->vy_target=RC_Ctl.rc.ch2 * 2.5;
		chassis_move->vw_target=RC_Ctl.rc.ch0 * 3;
	}
	//����Ǽ���ģʽ
	else if(chassis_move->chassis_mode == CHASSIS_KEYBOARD_MODE)
	{
		
	}
}

static void chassis_init(ChassisMove_t *chassis_move)
{
	unsigned char i;
	//ң��ģʽ
	chassis_move->chassis_mode = CHASSIS_RC_MODE;
	//����PID��ʼ��
	for(i=0; i<4; i++)
	{
		pid_struct_init(&chassis_move->motor_speed_pid[i], POSITION_PID, SPEED_LOOP, CHASSIS_MAX_PID_OUTER, CHASSIS_OUTER_INTEGRATION_LIMIT,
						CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD);
	}
	//
}

static void chassis_update_data(ChassisMove_t *chassis_update)
{
	unsigned char i;
	//x,y,yaw�ٶȸ��£���������ٶȸ�����CAN�ж���
	for(i=0; i<4; i++)
	{
		chassis_update->motor_speed_pid[i].measure	= chassis_update->chassis_motor[i].measure;
	}
	
	chassis_update->vx_measure = 0.25f * (chassis_update->chassis_motor[0].measure + chassis_update->chassis_motor[1].measure + chassis_update->chassis_motor[2].measure + chassis_update->chassis_motor[3].measure);
	chassis_update->vy_measure = 0.25f * (chassis_update->chassis_motor[0].measure - chassis_update->chassis_motor[1].measure - chassis_update->chassis_motor[2].measure + chassis_update->chassis_motor[3].measure);
	chassis_update->vw_measure = 0.25f * (chassis_update->chassis_motor[0].measure + chassis_update->chassis_motor[1].measure - chassis_update->chassis_motor[2].measure - chassis_update->chassis_motor[3].measure);
}

static void chassis_pid_calculate(ChassisMove_t *chassis_move)
{
	unsigned char i;
	short wheel_speed[4];
	mecanum_calculate(chassis_move->vx_target, chassis_move->vy_target, chassis_move->vw_target,wheel_speed);
	//����ÿ�����ӵ�Ŀ��ֵ
	chassis_move->motor_speed_pid[0].target = -wheel_speed[0];
	chassis_move->motor_speed_pid[1].target = -wheel_speed[1];
	chassis_move->motor_speed_pid[2].target = wheel_speed[2];
	//1��27�ĵ��
	chassis_move->motor_speed_pid[3].target = wheel_speed[3] * 27 / 19;
	
	//����ÿ�����ӵ��ٶ�ֵ
	for(i=0; i<4; i++)
	{
		pid_calculate(&chassis_move->motor_speed_pid[i],NULL);
	}
	
	//��ÿ�����ӵĵ���ֵ
	for(i=0; i<4; i++)
	{
		chassis_move->chassis_motor[i].give_current = (int16_t) chassis_move->motor_speed_pid[i].pos_out;
	}
}

static void mecanum_calculate(float vx,float vy,float vw, short *speed)
{
	u8 index;
	float buffer[4]={0},speed_max = 0 ,param;
	
	buffer[0] = vx + vy + vw;
	buffer[1] = vx - vy + vw;
	buffer[2] = vx - vy - vw;
	buffer[3] = vx + vy - vw;
	
	 for(index = 0, speed_max = 0; index < 4; index++)
    {
        if((buffer[index] > 0 ? buffer[index] : -buffer[index]) > speed_max)
        {
            speed_max = (buffer[index] > 0 ? buffer[index] : -buffer[index]);
        }
    }
    if(CHASSIS_MAX_SPEED < speed_max)
    {
        param = (float)CHASSIS_MAX_SPEED / speed_max;
        speed[0] = buffer[0] * param;
        speed[1] = buffer[1] * param;
        speed[2] = buffer[2] * param;
        speed[3] = buffer[3] * param; 
    }
    else
    {
        speed[0] = buffer[0];
        speed[1] = buffer[1];
        speed[2] = buffer[2];
        speed[3] = buffer[3];
    }
}

static void control_chassis_motor(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4)
{
	CanTxMsg CAN1_ChassisMotorStr;
	
	CAN1_ChassisMotorStr.StdId=CHASSIS_CONTROL_ID;
	CAN1_ChassisMotorStr.IDE=CAN_Id_Standard;
	CAN1_ChassisMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_ChassisMotorStr.DLC=0x08;
	
	CAN1_ChassisMotorStr.Data[0]=moto1 >> 8;
	CAN1_ChassisMotorStr.Data[1]=moto1;
	CAN1_ChassisMotorStr.Data[2]=moto2 >> 8;
	CAN1_ChassisMotorStr.Data[3]=moto2;
	CAN1_ChassisMotorStr.Data[4]=moto3 >> 8;
	CAN1_ChassisMotorStr.Data[5]=moto3;
	CAN1_ChassisMotorStr.Data[6]=moto4 >> 8;
	CAN1_ChassisMotorStr.Data[7]=moto4;
	
	CAN_Transmit(CAN1,&CAN1_ChassisMotorStr);
}
