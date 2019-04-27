/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{
// 
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#include <stdio.h>
#include <bsp_dma.h>
#include <task_led.h>
#include <task_imu.h>
#include <task_chassis.h>
#include <task_control.h>

#include <driver_dbus.h>
#include <driver_encoder.h>
#include <driver_control.h>


/*********************************************************************************************
 *********************************************************************************************/

u16 USART6_RXCnt=0;//串口6接受数据的长度


void USART6_DMA_SendData(u16 length)
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream7)){}//等待DMA可以被设置
	DMA_SetCurrDataCounter(DMA2_Stream7,length);
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7
														|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);//清除LISR和	
	DMA_Cmd(DMA2_Stream7,ENABLE);
}

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_IT_TCIF7))//判断传输完成
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7
															|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);//清除LISR和HISR寄存器
	}
}

void USART6_IRQHandler(void)
{
	u8 data=data;
	u8 i=0;
	if(USART_GetITStatus(USART6,USART_IT_IDLE))
	{
		data=USART6->SR;
		data=USART6->DR;//清除中断标志位
		
		DMA_Cmd(DMA2_Stream2,DISABLE);//防止传输数据
		USART6_RXCnt=RECEIVE_BUF_SIZE-DMA_GetCurrDataCounter(DMA2_Stream2);//总的缓存大小减去剩余缓存大小
		DMA_SetCurrDataCounter(DMA2_Stream2,RECEIVE_BUF_SIZE);//重新设置DMA的大小
		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2|DMA_FLAG_FEIF2|DMA_FLAG_DMEIF2
															|DMA_FLAG_TEIF2|DMA_FLAG_HTIF2);//清除LISR和HISR寄存器
		while(USART6_RXCnt--)
		{
			g_send_buff[i]=g_receive_buff[i];
			i++;
		}
//		data_judge();
		USART6_DMA_SendData(i);
		DMA_Cmd(DMA2_Stream2,ENABLE);
		LED1=!LED1;
	}
}

/*********************************************************************************************
 *********************************************************************************************/


void DMA2_Stream5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
	{
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
		
		RC_Ctl.rc.ch0 = (g_sbus_rx_buffer[0] | (g_sbus_rx_buffer[1] << 8)) & 0x07ff; 
		RC_Ctl.rc.ch0-= 1024;
		RC_Ctl.rc.ch1 = ((g_sbus_rx_buffer[1] >> 3) | (g_sbus_rx_buffer[2] << 5)) & 0x07ff; 
		RC_Ctl.rc.ch1-= 1024;
		RC_Ctl.rc.ch2 = ((g_sbus_rx_buffer[2] >> 6) | (g_sbus_rx_buffer[3] << 2) |	(g_sbus_rx_buffer[4] << 10)) & 0x07ff;
		RC_Ctl.rc.ch2-= 1024;
		RC_Ctl.rc.ch3 = ((g_sbus_rx_buffer[4] >> 1) | (g_sbus_rx_buffer[5] << 7)) & 0x07ff; 
		RC_Ctl.rc.ch3-= 1024;
		
		
		RC_Ctl.rc.s1 = ((g_sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; 				 //!< Switch left
		RC_Ctl.rc.s2 = ((g_sbus_rx_buffer[5] >> 4)& 0x0003); 							 //!< Switch right
		RC_Ctl.mouse.x = g_sbus_rx_buffer[6] | (g_sbus_rx_buffer[7] << 8); 	 //!< Mouse X axis
		RC_Ctl.mouse.y = g_sbus_rx_buffer[8] | (g_sbus_rx_buffer[9] << 8); 	 //!< Mouse Y axis
		RC_Ctl.mouse.z = g_sbus_rx_buffer[10] | (g_sbus_rx_buffer[11] << 8); //!< Mouse Z axis
		RC_Ctl.mouse.press_l = g_sbus_rx_buffer[12]; 											 //!< Mouse Left Is Press ?
		RC_Ctl.mouse.press_r = g_sbus_rx_buffer[13]; 											 //!< Mouse Right Is Press ?
		RC_Ctl.key.v = g_sbus_rx_buffer[14] | (g_sbus_rx_buffer[15] << 8); 	 //!< KeyBoard valu
	}
	//printf("%d\r\n",RC_Ctl.rc.ch0);
}

/*********************************************************************************************
 *********************************************************************************************/

CanRxMsg g_can1_receive_str;

MotoMeasure_t moto_chassis[4];
MotoMeasure_t moto_arm_left;
MotoMeasure_t moto_arm_right;
MotoMeasure_t moto_claw_left;
MotoMeasure_t moto_claw_right;


void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

void CAN1_RX0_IRQHandler(void)
{
	CAN_Receive(CAN1,CAN_FIFO0,&g_can1_receive_str);
	switch(g_can1_receive_str.StdId)
	{
		case 0x201:
		{
			encoder_data_handler(&moto_chassis[0], &g_can1_receive_str);
			g_chassis_move.chassis_motor[0].measure=moto_chassis[0].speed_rpm;
			break;
		}
		case 0x202:
		{
			encoder_data_handler(&moto_chassis[1], &g_can1_receive_str);
			g_chassis_move.chassis_motor[1].measure=moto_chassis[1].speed_rpm;
			break;
		}
		case 0x203:
		{
			encoder_data_handler(&moto_chassis[2], &g_can1_receive_str);
			g_chassis_move.chassis_motor[2].measure=moto_chassis[2].speed_rpm;
			break;
		}
		case 0x204:
		{
			encoder_data_handler(&moto_chassis[3], &g_can1_receive_str);
			g_chassis_move.chassis_motor[3].measure=moto_chassis[3].speed_rpm;
			break;
		}
		
		case 0x205:
		{
			encoder_data_handler(&moto_arm_left,&g_can1_receive_str);
			g_rise_arm_left_pid.measure = moto_arm_left.speed_rpm;
			g_get_bullet.arm_moto_left.round = - moto_arm_left.round_cnt;
			break;
		}
		
		
		case 0x206: 
		{
			encoder_data_handler(&moto_arm_right,&g_can1_receive_str);
			g_rise_arm_right_pid.measure = moto_arm_right.speed_rpm;
			g_get_bullet.arm_moto_right.round =  moto_arm_right.round_cnt;
			break;
		}
		
		
		case 0x207:
		{
			encoder_data_handler(&moto_claw_left,&g_can1_receive_str);
			//速度更新
			g_get_bullet.claw_moto_left.moto_speed_pid.measure = moto_claw_left.speed_rpm;
			//2006电机圈数,最多36圈
			if(moto_claw_left.round_cnt == 36) moto_claw_left.round_cnt =0;
			g_get_bullet.claw_moto_left.current_round = moto_claw_left.round_cnt;
			g_get_bullet.claw_moto_left.cuttent_ecd = moto_claw_left.ecd;
			g_get_bullet.claw_moto_left.moto_angle_pid.measure = (g_get_bullet.claw_moto_left.current_round * 8191 + g_get_bullet.claw_moto_left.cuttent_ecd) * MOTOR_ECD_TO_DEG;
			break;
		}
	
		case 0x208:
		{
			encoder_data_handler(&moto_claw_right,&g_can1_receive_str);
			//速度更新
			g_get_bullet.claw_moto_right.moto_speed_pid.measure = moto_claw_right.speed_rpm;
			//2006电机圈数,最多36圈
			if(moto_claw_right.round_cnt == 36) moto_claw_right.round_cnt =0;
			g_get_bullet.claw_moto_right.current_round = moto_claw_right.round_cnt;
			g_get_bullet.claw_moto_right.cuttent_ecd = moto_claw_right.ecd;
			g_get_bullet.claw_moto_right.moto_angle_pid.measure = (g_get_bullet.claw_moto_right.current_round * 8191 + g_get_bullet.claw_moto_right.cuttent_ecd) * MOTOR_ECD_TO_DEG;
			break;
		}
		default:;
			printf("No feedback speed\r\n");
	}
//	for(i=0;i<CAN1_ReceiveStr.DLC;i++)
//	CAN1_ReceiveBuffer[i]=CAN1_ReceiveStr.Data[i];
	
//	printf("%d\r\n",(g_can1_receive_str.Data[0]<<8)|g_can1_receive_str.Data[1]);
//	printf("%d\r\n",(g_can1_receive_str.Data[2]<<8)|g_can1_receive_str.Data[3]);
//	printf("%d\r\n",(CAN1_ReceiveStr.Data[4]<<8)|CAN1_ReceiveStr.Data[5]);
}

/*********************************************************************************************
 *********************************************************************************************/

u8 CAN2_ReceiveBuffer[64];

void CAN2_RX1_IRQHandler(void)
{
	u8 i;
	CanRxMsg ReceiveStr;
	CAN_Receive(CAN2,CAN_FIFO1,&ReceiveStr);	
	for(i=0;i<ReceiveStr.DLC;i++)
	CAN2_ReceiveBuffer[i]=ReceiveStr.Data[i];
}
