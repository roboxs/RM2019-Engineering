#include <bsp_nvic.h>

void BSP_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_Structure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//USART6(裁判系统)
	NVIC_Structure.NVIC_IRQChannel					= USART6_IRQn;
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_Structure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_Structure);
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	
	//USART6->DMA发送完成中断
	NVIC_Structure.NVIC_IRQChannel 					= DMA2_Stream7_IRQn;  	
	NVIC_Structure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;  
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_Init(&NVIC_Structure);  
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	
	//DBUS->DMA发送完成中断
	NVIC_Structure.NVIC_IRQChannel 					= DMA2_Stream5_IRQn;		
	NVIC_Structure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 11;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
	
	//CAN1->FIFO 0 消息挂起中断
	NVIC_Structure.NVIC_IRQChannel					= CAN1_RX0_IRQn;	
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO 0 消息挂起中断
	
	//CAN2->FIF0 1 消息挂起中断
	NVIC_Structure.NVIC_IRQChannel					= CAN2_RX1_IRQn;
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 11;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO 1消息挂起中断
}
