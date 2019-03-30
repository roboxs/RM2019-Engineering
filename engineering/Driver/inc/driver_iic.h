#ifndef _DRIVER_IIC_H
#define _DRIVER_IIC_H

#include<stm32f4xx.h>
#include<sys.h>


//IO方向设置
#define SDA_IN()  {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=0<<9*2;}	//PF9输入模式
#define SDA_OUT() {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=1<<9*2;} //PF9输出模式
//IO操作函数	 
#define IIC_SCL    PFout(7) //SCL
#define IIC_SDA    PFout(9) //SDA	 
#define READ_SDA   PFin(9)  //输入SDA 


void IIC_Init(void);			 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

#endif 
