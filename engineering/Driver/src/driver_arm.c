#include <driver_arm.h>
#include <driver_dbus.h>
#include <driver_encoder.h>


#define ABS(x)	((x>0) ? (x) : (-x))


void set_motor_current(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4)
{
	CanTxMsg CAN1_ChassisMotorStr;
	
	CAN1_ChassisMotorStr.StdId=ARM_CONTROL_ID;
	CAN1_ChassisMotorStr.IDE=CAN_Id_Standard;
	CAN1_ChassisMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_ChassisMotorStr.DLC=0x08;
	
	CAN1_ChassisMotorStr.Data[0]=(u8)(moto1 >> 8);
	CAN1_ChassisMotorStr.Data[1]=(u8)moto1;
	CAN1_ChassisMotorStr.Data[2]=(u8)(moto2 >> 8);
	CAN1_ChassisMotorStr.Data[3]=(u8)moto2;
	CAN1_ChassisMotorStr.Data[4]=(u8)(moto3 >> 8);
	CAN1_ChassisMotorStr.Data[5]=(u8)moto3;
	CAN1_ChassisMotorStr.Data[6]=(u8)(moto4 >> 8);
	CAN1_ChassisMotorStr.Data[7]=(u8)moto4;
	
	CAN_Transmit(CAN1,&CAN1_ChassisMotorStr);
}

void test_3510_motor(void)
{
	CanTxMsg CAN1_ChassisMotorStr;
	
	CAN1_ChassisMotorStr.StdId=ARM_CONTROL_ID;
	CAN1_ChassisMotorStr.IDE=CAN_Id_Standard;
	CAN1_ChassisMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_ChassisMotorStr.DLC=0x08;
	
	CAN1_ChassisMotorStr.Data[1]=0x01;
	CAN1_ChassisMotorStr.Data[0]=0xf4;
	
	CAN1_ChassisMotorStr.Data[3]=0x05;
	CAN1_ChassisMotorStr.Data[2]=0xDC;
	
	CAN1_ChassisMotorStr.Data[5]=0x05;
	CAN1_ChassisMotorStr.Data[4]=0xDC;
	
	CAN1_ChassisMotorStr.Data[7]=0x05;
	CAN1_ChassisMotorStr.Data[6]=0xDC;
	
	CAN_Transmit(CAN1,&CAN1_ChassisMotorStr);
}


