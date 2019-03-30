#include <driver_claw.h>
#include <driver_control.h>
#include <task_led.h>




void control_2006_motor(int16_t moto1,int16_t moto2)
{
	CanTxMsg CAN1_DialMotorStr;
	
	CAN1_DialMotorStr.StdId=CLAW_CONTROL_ID;
	CAN1_DialMotorStr.IDE=CAN_Id_Standard;
	CAN1_DialMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_DialMotorStr.DLC=0x08;
	
	CAN1_DialMotorStr.Data[0]=0;
	CAN1_DialMotorStr.Data[1]=0;
	CAN1_DialMotorStr.Data[2]=0;
	CAN1_DialMotorStr.Data[3]=0;
	CAN1_DialMotorStr.Data[4]=(moto1 >> 8);
	CAN1_DialMotorStr.Data[5]=moto1;
	CAN1_DialMotorStr.Data[6]=(moto2 >> 8);
	CAN1_DialMotorStr.Data[7]=moto2;
	
	CAN_Transmit(CAN1,&CAN1_DialMotorStr);
	
}

