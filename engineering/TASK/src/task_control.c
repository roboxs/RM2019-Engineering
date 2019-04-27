#include <delay.h>
#include <task_control.h>
#include <task_led.h>
#include <driver_control.h>
#include <driver_claw.h>
#include <driver_dbus.h>
#include <driver_arm.h>
#include <stm32f4xx_it.h>
#include <driver_island.h>
#include <task_chassis.h>

GetBullet_t g_get_bullet={0};
static int g_control_count=0;
static int g_island_count=0;
float test_target =1000;
float angle_target = 60;

float test_kp = 5,test_ki = 0.1,test_kd=0;
void control_task(void *pvParameters)
{
	//��ǰ����ĳ�ʼ��
	control_task_init();
	//���µ����������
	xTaskCreate(set_motor_current_task, "set_motor_current_task", 	256, 	 NULL, 3, NULL);
	while(1)
	{
		switch(RC_Ctl.rc.s1)//�󿪹�
		{
			//̧������
			case 1:
				//move_claw_task();
				break;
			//��ȡ����
			case 3:
				get_buffet_task();
				break;
			case 2://צ�Ӹ�λ
			{
				if( (RC_Ctl.rc.s2) == 0x02)//��ȫ��λ
				{
					g_get_bullet.arm_finsh_flag = 0;//��λ��־λ
					g_control_count =0;//������λ
					//̧����λ����
					if( (g_get_bullet.arm_moto_left.round >= 0) && (g_get_bullet.arm_moto_right.round >= 0) )
					{
						g_get_bullet.arm_moto_left.target =	  ARM_NORMAL_SPEED;
						g_get_bullet.arm_moto_right.target = -ARM_NORMAL_SPEED;
					}
					else 
					{
						g_get_bullet.arm_moto_left.target = ARM_MIN_SPEED;
						g_get_bullet.arm_moto_right.target =- ARM_MIN_SPEED;
					}
				}
				else
				{
					g_control_count =0;//������λ
				}
			}
			
		}
		
		switch(RC_Ctl.rc.s2)//�ҿ���
		{
			case 1:
				up_island_task();
				break;
			case 3:
				move_push_motor(-1000,-1000);
				move_assist_motor(0,0);
				g_island_count=0;
				break;
			case 2:
			down_island_task();
				break;
		}
		
		LED5=!LED5;
//		pid_reset(&(g_get_bullet.claw_moto_right.moto_speed_pid),test_kp,test_ki,test_kd);
		vTaskDelay(CONTROL_TASK_CYCLE);
	}
}

//���������ʼ��
static void control_task_init(void)
{
/*claw-left-2006-outer*/pid_struct_init(&(g_get_bullet.claw_moto_left.moto_speed_pid), POSITION_PID, DOUBLE_LOOP, MOVE_CLAW_INNER_MAXOUT, MOVE_CLAW_INNER_INTEGRATION_LIMIT,
					MOVE_CLAW_INNER_P,  MOVE_CLAW_INNER_I , MOVE_CLAW_INNER_D);
/*claw-left-2006-inner*/pid_struct_init(&(g_get_bullet.claw_moto_left.moto_angle_pid), POSITION_PID, DOUBLE_LOOP, MOVE_CLAW_OUTER_MAXOUT, MOVE_CLAW_OUTER_INTEGRATION_LIMIT,
					MOVE_CLAW_OUTER_P,  MOVE_CLAW_OUTER_I , MOVE_CLAW_OUTER_D);

/*claw-right-2006-outer*/pid_struct_init(&(g_get_bullet.claw_moto_right.moto_speed_pid), POSITION_PID, DOUBLE_LOOP, MOVE_CLAW_INNER_MAXOUT, MOVE_CLAW_INNER_INTEGRATION_LIMIT,
					MOVE_CLAW_INNER_P,  MOVE_CLAW_INNER_I , MOVE_CLAW_INNER_D);
/*claw-right-2006-inner*/pid_struct_init(&(g_get_bullet.claw_moto_right.moto_angle_pid), POSITION_PID, DOUBLE_LOOP, MOVE_CLAW_OUTER_MAXOUT, MOVE_CLAW_OUTER_INTEGRATION_LIMIT,
					MOVE_CLAW_OUTER_P,  MOVE_CLAW_OUTER_I , MOVE_CLAW_OUTER_D);	
}	


//��ȡ�ӵ�����
static void get_buffet_task(void)
{
		//��̧������δ���ʱ
		if(g_get_bullet.arm_finsh_flag != SET)
		{
			g_get_bullet.arm_moto_left.target  = -ARM_NORMAL_SPEED;
			g_get_bullet.arm_moto_right.target = ARM_NORMAL_SPEED;
			//��̧��������г�ʱ
			if((g_get_bullet.arm_moto_left.round >= ARM_TOTAL_DISTANCE) || (g_get_bullet.arm_moto_right.round >= ARM_TOTAL_DISTANCE))
			{
				//̧���������
				g_get_bullet.arm_finsh_flag = 1;
			}
		}
		//̧��������ɺ�
		else
		{
			g_get_bullet.arm_moto_left.target  =  -ARM_MIN_SPEED;
			g_get_bullet.arm_moto_right.target = ARM_MIN_SPEED;
		}
}

////צ���˶�
//static void move_claw_task(void)
//{
//	switch(g_control_count)
//	{
//		case TAKE_BULLET_START :  //�ƽ� ---��ʱ1500ms
//			POWER_PUSH_CTRL = 1;
//			break;
//		case 150://���� ---��ʱ1500ms
//			g_get_bullet.claw_moto_left.target  =  -CLAW_NORMAL_SPEED;
//			g_get_bullet.claw_moto_right.target = CLAW_NORMAL_SPEED;
//			break;
//		case 300://��ȡ ---��ʱ500ms
//			g_get_bullet.claw_moto_left.target = CLAW_MIN_SPEED;
//			g_get_bullet.claw_moto_right.target = CLAW_MIN_SPEED;
//			POWER_CLAMP_CTRL = 1;
//			break;
//		case 350://�˻�  ---1500ms
//				POWER_PUSH_CTRL =0 ;
//			break;
//		case 500://���� ---��ʱ1500ms
//			g_get_bullet.claw_moto_left.target = CLAW_NORMAL_SPEED;
//			g_get_bullet.claw_moto_right.target = -CLAW_NORMAL_SPEED;
//			break;
//		case 650://���������ּ�ȡ״̬�� ---��ʱ1000ms
//			g_get_bullet.claw_moto_left.target  =  -CLAW_NORMAL_SPEED;
//			g_get_bullet.claw_moto_right.target = CLAW_NORMAL_SPEED;
//			break;
//		case 730:
//			POWER_CLAMP_CTRL = 0;//�ɿ���ҩ��
//			break;
//		case 750://���� ---��ʱ1000ms
//			g_get_bullet.claw_moto_left.target  =  CLAW_NORMAL_SPEED;
//			g_get_bullet.claw_moto_right.target =  -CLAW_NORMAL_SPEED;
//			break;
//		case 820://���ͣת,�˻�
//			g_get_bullet.claw_moto_left.target  =  CLAW_MIN_SPEED;
//			g_get_bullet.claw_moto_right.target =  CLAW_MIN_SPEED;
//			POWER_PUSH_CTRL =0;
//			break;
//	}
//	g_control_count++;
//}

static void up_island_task(void)
{	
	switch(g_island_count)
	{
		case 0:
			move_push_motor(1000,1000);	
			break;
		case 400:
			move_push_motor(0,0);
			move_assist_motor(1000,1000);
			break;
		case 600:
			//move_assist_motor(0,0);
			g_chassis_move.vx_target = 1500;
			break;
		case 1100:
			g_chassis_move.vx_target = 0;
			move_push_motor(-1000,-1000);
			move_assist_motor(0, 0);
			break;
		case 1400:
			move_push_motor(0,0);
			
	}
	g_island_count++;
}

static void down_island_task(void)
{
	switch(g_island_count)
	{
		case 0:	//�����ֲ����½� --3000ms
			move_push_motor(1000,1000);
			break;
		case 150: //����2000ms
			move_push_motor(0,0);
			break;
		case 500://����������
			move_push_motor(-1000,-1000);
			break;
		case 650:
			break;
	}
	g_island_count++;
}


static void bullet_pid_calculate(GetBullet_t * bullet)
{
	//̧�����Ŀ��ֵ
	g_rise_arm_left_pid.target = bullet->arm_moto_left.target;
	g_rise_arm_right_pid.target = bullet->arm_moto_right.target;
	//צ�ӵ��Ŀ��ֵ
	bullet->claw_moto_left.moto_angle_pid.target = 60;
	bullet->claw_moto_right.moto_angle_pid.target = angle_target;
	bullet->claw_moto_right.moto_speed_pid.target = test_target;
	//pid����
	pid_calculate(&g_rise_arm_left_pid, NULL);
	pid_calculate(&g_rise_arm_right_pid, NULL);
	pid_calculate(&(bullet->claw_moto_left.moto_speed_pid), &(bullet->claw_moto_left.moto_angle_pid));
	pid_calculate(&(bullet->claw_moto_right.moto_speed_pid), &(bullet->claw_moto_right.moto_angle_pid));
	//���������ֵ
	bullet->arm_moto_left.give_current = g_rise_arm_left_pid.pos_out;
	bullet->arm_moto_right.give_current = g_rise_arm_right_pid.pos_out;
	bullet->claw_moto_left.give_current = bullet->claw_moto_left.moto_speed_pid.pos_out;
	bullet->claw_moto_right.give_current = bullet->claw_moto_right.moto_speed_pid.pos_out;

}

static void set_motor_current_task(void *pvParameters)
{
	while(1)
	{
		//ȡ������pid����
		bullet_pid_calculate(&g_get_bullet);
		//ȡ���������е�������趨
		set_motor_current(g_get_bullet.arm_moto_left.give_current, g_get_bullet.arm_moto_right.give_current,
						  g_get_bullet.claw_moto_left.give_current,g_get_bullet.claw_moto_right.give_current);
		LED6=!LED6;
		vTaskDelay(UPDATE_DATA_CYCLE);
	}
}

